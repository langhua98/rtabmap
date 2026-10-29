//
//  CoverageRadar.swift
//  RTABMapApp
//
//  扫描覆盖雷达：在扫描时用 LiDAR 深度图统计哪些地方已经扫到，
//  画成一个以手机为中心、朝向向上的俯视小地图，并给出文字提示。
//

import ARKit
import UIKit
import simd

/// Accumulates which parts of the room the LiDAR has seen, on a top-down grid.
final class CoverageTracker {
    /// Grid cell size, in meters.
    static let cellSize: Float = 0.2
    /// Samples a cell needs before it counts as well scanned.
    static let wellScannedHits: UInt16 = 4
    static let sectorCount = 12

    struct Cell: Hashable {
        var x: Int32
        var z: Int32
    }

    /// Walls and furniture: points around the phone's height.
    private(set) var wallHits: [Cell: UInt16] = [:]
    /// Floor: points well below the phone.
    private(set) var floorCells: Set<Cell> = []
    private(set) var ceilingSamples = 0
    private(set) var cameraPosition = SIMD3<Float>(repeating: 0)
    /// Unit vectors of the camera's forward and right directions on the floor plane (x, z).
    private(set) var forward = SIMD2<Float>(0, -1)
    private(set) var right = SIMD2<Float>(1, 0)
    private(set) var medianDepth: Float = 0
    private(set) var frameCount = 0
    private var lastUpdate: TimeInterval = 0
    /// Same filter as the map itself (Settings → 扫描 → 深度置信度), so the radar only
    /// counts points that will end up in the model.
    private var minConfidence = UInt8(ARConfidenceLevel.medium.rawValue)
    /// Floor cells inside the scanned room that have not been seen yet, refreshed by `add(frame:)`.
    private(set) var missingFloorCells: [Cell] = []
    private(set) var floorCoverage: Float = 0

    func reset() {
        minConfidence = UInt8(clamping: UserDefaults.standard.integer(forKey: "DepthConfidence"))
        missingFloorCells = []
        floorCoverage = 0
        wallHits.removeAll()
        floorCells.removeAll()
        ceilingSamples = 0
        medianDepth = 0
        frameCount = 0
        lastUpdate = 0
    }

    /// Adds the depth samples of one frame. Throttled to 5 frames per second.
    /// Returns false when the frame was skipped.
    @discardableResult
    func add(frame: ARFrame) -> Bool {
        guard frame.timestamp - lastUpdate >= 0.2 else { return false }
        guard case .normal = frame.camera.trackingState else { return false }
        guard let depth = frame.sceneDepth else { return false }
        lastUpdate = frame.timestamp

        let transform = frame.camera.transform
        cameraPosition = SIMD3(transform.columns.3.x, transform.columns.3.y, transform.columns.3.z)
        let back = SIMD2(transform.columns.2.x, transform.columns.2.z)
        let side = SIMD2(transform.columns.0.x, transform.columns.0.z)
        // Looking straight down or up leaves no horizontal heading: keep the last one.
        if simd_length(back) > 0.2 {
            forward = simd_normalize(-back)
            right = SIMD2(-forward.y, forward.x)
            if simd_dot(right, side) < 0 { right = -right }
        }

        let depthMap = depth.depthMap
        let confidenceMap = depth.confidenceMap
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        if let confidenceMap { CVPixelBufferLockBaseAddress(confidenceMap, .readOnly) }
        defer { if let confidenceMap { CVPixelBufferUnlockBaseAddress(confidenceMap, .readOnly) } }

        guard let depthBase = CVPixelBufferGetBaseAddress(depthMap) else { return false }
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let depthRowBytes = CVPixelBufferGetBytesPerRow(depthMap)
        let confidenceBase = confidenceMap.flatMap { CVPixelBufferGetBaseAddress($0) }
        let confidenceRowBytes = confidenceMap.map { CVPixelBufferGetBytesPerRow($0) } ?? 0

        // Intrinsics are given for the color image; scale them to the depth map.
        let scale = Float(width) / Float(frame.camera.imageResolution.width)
        let k = frame.camera.intrinsics
        let fx = k[0][0] * scale, fy = k[1][1] * scale
        let cx = k[2][0] * scale, cy = k[2][1] * scale

        let columns = 24, rows = 18
        var depths: [Float] = []
        depths.reserveCapacity(columns * rows)
        var seenThisFrame = Set<Cell>()

        for row in 0..<rows {
            let v = (row * height + height / 2) / rows
            let depthRow = depthBase.advanced(by: v * depthRowBytes).assumingMemoryBound(to: Float32.self)
            for column in 0..<columns {
                let u = (column * width + width / 2) / columns
                if let confidenceBase {
                    let confidence = confidenceBase.advanced(by: v * confidenceRowBytes)
                        .assumingMemoryBound(to: UInt8.self)[u]
                    if confidence < minConfidence { continue }
                }
                let d = depthRow[u]
                guard d.isFinite, d > 0.1, d < 5 else { continue }
                depths.append(d)

                // Image coordinates (x right, y down) to ARKit camera space (x right, y up, looking at -z).
                let local = SIMD4<Float>((Float(u) - cx) * d / fx, -(Float(v) - cy) * d / fy, -d, 1)
                let world = transform * local
                let cell = Cell(x: Int32((world.x / Self.cellSize).rounded(.down)),
                                z: Int32((world.z / Self.cellSize).rounded(.down)))
                let relativeHeight = world.y - cameraPosition.y
                if relativeHeight < -1.0 {
                    floorCells.insert(cell)
                } else if relativeHeight > 0.9 {
                    ceilingSamples += 1
                } else if seenThisFrame.insert(cell).inserted {
                    // Count each cell once per frame, so hits mean "seen in N frames".
                    wallHits[cell] = min((wallHits[cell] ?? 0) + 1, 10_000)
                }
            }
        }

        if !depths.isEmpty {
            depths.sort()
            medianDepth = depths[depths.count / 2]
        }
        frameCount += 1
        updateFloorCoverage()
        return true
    }

    /// The room is the box around the well-scanned walls; every cell inside it should
    /// have floor under it, except where furniture stands.
    private func updateFloorCoverage() {
        let walls = wallHits.filter { $0.value >= Self.wellScannedHits }.map(\.key)
        guard let minX = walls.map(\.x).min(), let maxX = walls.map(\.x).max(),
              let minZ = walls.map(\.z).min(), let maxZ = walls.map(\.z).max(),
              maxX - minX >= 4, maxZ - minZ >= 4, maxX - minX <= 100, maxZ - minZ <= 100 else {
            missingFloorCells = []
            floorCoverage = 0
            return
        }
        var missing: [Cell] = []
        var total = 0
        for x in (minX + 1)..<maxX {
            for z in (minZ + 1)..<maxZ {
                let cell = Cell(x: x, z: z)
                if (wallHits[cell] ?? 0) >= Self.wellScannedHits { continue }
                total += 1
                if !floorCells.contains(cell) { missing.append(cell) }
            }
        }
        missingFloorCells = missing
        floorCoverage = total > 0 ? 1 - Float(missing.count) / Float(total) : 0
    }

    /// For each of `sectorCount` world directions around the phone (within 6 m):
    /// true when walls or furniture in that direction have been scanned well.
    func sectorCoverage() -> [Bool] {
        var totals = [Int](repeating: 0, count: Self.sectorCount)
        for (cell, hits) in wallHits where hits >= Self.wellScannedHits {
            let offset = center(of: cell) - SIMD2(cameraPosition.x, cameraPosition.z)
            let distance = simd_length(offset)
            guard distance > 0.3, distance < 6 else { continue }
            totals[sector(ofWorldDirection: offset)] += 1
        }
        return totals.map { $0 >= 3 }
    }

    func center(of cell: Cell) -> SIMD2<Float> {
        SIMD2((Float(cell.x) + 0.5) * Self.cellSize, (Float(cell.z) + 0.5) * Self.cellSize)
    }

    func sector(ofWorldDirection direction: SIMD2<Float>) -> Int {
        let angle = atan2(direction.y, direction.x) // -pi...pi
        let fraction = (angle + .pi) / (2 * .pi)
        return min(Int(fraction * Float(Self.sectorCount)), Self.sectorCount - 1)
    }

    /// World direction (x, z) through the middle of a sector.
    func direction(ofSector sector: Int) -> SIMD2<Float> {
        let angle = (Float(sector) + 0.5) / Float(Self.sectorCount) * 2 * .pi - .pi
        return SIMD2(cos(angle), sin(angle))
    }

    /// Converts a world direction to the phone's frame: x to the right, y forward.
    func relativeToPhone(_ direction: SIMD2<Float>) -> SIMD2<Float> {
        SIMD2(simd_dot(direction, right), simd_dot(direction, forward))
    }

    /// 前方, 右后方, … for a world direction, as seen from the phone.
    func directionName(_ direction: SIMD2<Float>) -> String {
        let relative = relativeToPhone(direction)
        let degrees = atan2(relative.x, relative.y) * 180 / .pi // 0 = ahead, 90 = right
        let names = ["前方", "右前方", "右侧", "右后方", "后方", "左后方", "左侧", "左前方"]
        let index = Int(((degrees + 360 + 22.5).truncatingRemainder(dividingBy: 360)) / 45)
        return names[min(index, names.count - 1)]
    }

    /// One line telling the user what to do next.
    func hint() -> String {
        if frameCount < 5 {
            return "慢慢移动手机，对准墙面和家具"
        }
        if medianDepth > 0, medianDepth < 0.35 {
            return "离得太近了，往后退一点"
        }
        if medianDepth > 3.5 {
            return "离得太远了，靠近到 3 米以内效果更好"
        }
        let coverage = sectorCoverage()
        let missing = coverage.indices.filter { !coverage[$0] }
        if let nearest = missing.min(by: { turnAngle(toSector: $0) < turnAngle(toSector: $1) }) {
            return "\(directionName(direction(ofSector: nearest)))还没扫到，转过去扫一下"
        }
        if floorCoverage < 0.85 {
            return "地面还有空缺（雷达上的红格子），低头补扫一下"
        }
        if ceilingSamples < 300 {
            return "四周扫好了，再抬头扫一下天花板"
        }
        return "都扫到了，可以点停止保存"
    }

    private func turnAngle(toSector sector: Int) -> Float {
        let relative = relativeToPhone(direction(ofSector: sector))
        return abs(atan2(relative.x, relative.y))
    }
}

/// Heading-up, top-down mini map of what has been scanned, centered on the phone.
final class CoverageRadarView: UIView {
    /// Meters from the center to the edge of the radar.
    var range: Float = 5
    private var tracker: CoverageTracker?

    private let hintLabel = UILabel()
    private let badgeLabel = UILabel()

    override init(frame: CGRect) {
        super.init(frame: frame)
        isOpaque = false
        isUserInteractionEnabled = false
        backgroundColor = .clear
    }

    required init?(coder: NSCoder) {
        super.init(coder: coder)
    }

    /// Adds the radar and its hint text to the bottom-left corner of `parent`.
    static func install(in parent: UIView) -> CoverageRadarView {
        let radar = CoverageRadarView(frame: .zero)
        radar.translatesAutoresizingMaskIntoConstraints = false
        parent.addSubview(radar)

        for label in [radar.hintLabel, radar.badgeLabel] {
            label.translatesAutoresizingMaskIntoConstraints = false
            label.textColor = .white
            label.numberOfLines = 0
            label.layer.shadowColor = UIColor.black.cgColor
            label.layer.shadowRadius = 2
            label.layer.shadowOpacity = 1
            label.layer.shadowOffset = .zero
            parent.addSubview(label)
        }
        radar.hintLabel.font = .boldSystemFont(ofSize: 17)
        radar.badgeLabel.font = .systemFont(ofSize: 13)

        let guide = parent.safeAreaLayoutGuide
        NSLayoutConstraint.activate([
            radar.leadingAnchor.constraint(equalTo: guide.leadingAnchor, constant: 16),
            radar.bottomAnchor.constraint(equalTo: guide.bottomAnchor, constant: -24),
            radar.widthAnchor.constraint(equalToConstant: 150),
            radar.heightAnchor.constraint(equalToConstant: 150),
            radar.badgeLabel.leadingAnchor.constraint(equalTo: radar.leadingAnchor),
            radar.badgeLabel.bottomAnchor.constraint(equalTo: radar.topAnchor, constant: -6),
            radar.hintLabel.leadingAnchor.constraint(equalTo: radar.leadingAnchor),
            radar.hintLabel.bottomAnchor.constraint(equalTo: radar.badgeLabel.topAnchor, constant: -4),
            radar.hintLabel.trailingAnchor.constraint(lessThanOrEqualTo: guide.trailingAnchor, constant: -90),
        ])
        radar.isHidden = true
        return radar
    }

    override var isHidden: Bool {
        didSet {
            hintLabel.isHidden = isHidden
            badgeLabel.isHidden = isHidden
        }
    }

    func update(with tracker: CoverageTracker) {
        self.tracker = tracker
        hintLabel.text = tracker.hint()
        let floor = "\(Int((tracker.floorCoverage * 100).rounded()))%"
        let ceiling = tracker.ceilingSamples >= 300 ? "已扫" : "未扫"
        badgeLabel.text = "地面：\(floor)    天花板：\(ceiling)"
        setNeedsDisplay()
    }

    override func draw(_ rect: CGRect) {
        guard let context = UIGraphicsGetCurrentContext() else { return }
        let radius = min(bounds.width, bounds.height) / 2 - 6
        let center = CGPoint(x: bounds.midX, y: bounds.midY)
        let pointsPerMeter = radius / CGFloat(range)

        // Background disc and distance rings every meter.
        let disc = UIBezierPath(arcCenter: center, radius: radius, startAngle: 0, endAngle: 2 * .pi, clockwise: true)
        UIColor.black.withAlphaComponent(0.45).setFill()
        disc.fill()
        context.saveGState()
        disc.addClip()

        UIColor.white.withAlphaComponent(0.15).setStroke()
        for meters in stride(from: 1, to: Int(range), by: 1) {
            let ring = UIBezierPath(arcCenter: center, radius: CGFloat(meters) * pointsPerMeter,
                                    startAngle: 0, endAngle: 2 * .pi, clockwise: true)
            ring.lineWidth = 0.5
            ring.stroke()
        }

        if let tracker {
            let origin = SIMD2(tracker.cameraPosition.x, tracker.cameraPosition.z)
            let cellPoints = max(CGFloat(CoverageTracker.cellSize) * pointsPerMeter, 1.5)

            func point(for cell: CoverageTracker.Cell) -> CGPoint? {
                let offset = tracker.center(of: cell) - origin
                guard simd_length(offset) <= range else { return nil }
                let relative = tracker.relativeToPhone(offset)
                return CGPoint(x: center.x + CGFloat(relative.x) * pointsPerMeter,
                               y: center.y - CGFloat(relative.y) * pointsPerMeter)
            }

            // Scanned floor: faint blue.
            UIColor.systemBlue.withAlphaComponent(0.35).setFill()
            for cell in tracker.floorCells {
                if let p = point(for: cell) {
                    context.fill(CGRect(x: p.x - cellPoints / 2, y: p.y - cellPoints / 2, width: cellPoints, height: cellPoints))
                }
            }
            // Floor inside the room that has not been seen yet: faint red.
            UIColor.systemRed.withAlphaComponent(0.4).setFill()
            for cell in tracker.missingFloorCells {
                if let p = point(for: cell) {
                    context.fill(CGRect(x: p.x - cellPoints / 2, y: p.y - cellPoints / 2, width: cellPoints, height: cellPoints))
                }
            }
            // Walls and furniture: yellow when seen a little, green when scanned well.
            for (cell, hits) in tracker.wallHits {
                guard let p = point(for: cell) else { continue }
                (hits >= CoverageTracker.wellScannedHits ? UIColor.systemGreen : UIColor.systemYellow).setFill()
                context.fill(CGRect(x: p.x - cellPoints / 2, y: p.y - cellPoints / 2, width: cellPoints, height: cellPoints))
            }

            // Outer ring: green where that direction is scanned, red where it is not.
            // Sector edges are fixed in world space, so the arcs stay put while the phone turns.
            for (sector, covered) in tracker.sectorCoverage().enumerated() {
                let edges = [sector, sector + 1].map { edge -> CGFloat in
                    let angle = Float(edge) / Float(CoverageTracker.sectorCount) * 2 * .pi - .pi
                    let relative = tracker.relativeToPhone(SIMD2(cos(angle), sin(angle)))
                    return CGFloat(atan2(-relative.y, relative.x))
                }
                // Draw the short way between the two edges, whatever the winding.
                var sweep = edges[1] - edges[0]
                while sweep > .pi { sweep -= 2 * .pi }
                while sweep < -.pi { sweep += 2 * .pi }
                let arc = UIBezierPath(arcCenter: center, radius: radius - 3, startAngle: edges[0],
                                       endAngle: edges[0] + sweep, clockwise: sweep > 0)
                arc.lineWidth = 5
                (covered ? UIColor.systemGreen : UIColor.systemRed).withAlphaComponent(0.85).setStroke()
                arc.stroke()
            }
        }
        context.restoreGState()

        // The phone, pointing up.
        let arrow = UIBezierPath()
        arrow.move(to: CGPoint(x: center.x, y: center.y - 9))
        arrow.addLine(to: CGPoint(x: center.x + 6, y: center.y + 6))
        arrow.addLine(to: CGPoint(x: center.x, y: center.y + 2))
        arrow.addLine(to: CGPoint(x: center.x - 6, y: center.y + 6))
        arrow.close()
        UIColor.white.setFill()
        arrow.fill()

        UIColor.white.withAlphaComponent(0.6).setStroke()
        disc.lineWidth = 1
        disc.stroke()
    }
}
