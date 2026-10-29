//
//  AppDelegate.swift
//  GLKittutorial
//
//  Created by Mathieu Labbe on 2020-12-28.
//

import UIKit
import ARKit

func setDefaultsFromSettingsBundle() {
    
    let plistFiles = ["Root", "Mapping", "Assembling"]
    
    for plistName in plistFiles {
        //Read PreferenceSpecifiers from Root.plist in Settings.Bundle
        if let settingsURL = Bundle.main.url(forResource: plistName, withExtension: "plist", subdirectory: "Settings.bundle"),
            let settingsPlist = NSDictionary(contentsOf: settingsURL),
            let preferences = settingsPlist["PreferenceSpecifiers"] as? [NSDictionary] {

            for prefSpecification in preferences {

                if let key = prefSpecification["Key"] as? String, let value = prefSpecification["DefaultValue"] {

                    //If key doesn't exists in userDefaults then register it, else keep original value
                    if UserDefaults.standard.value(forKey: key) == nil {

                        UserDefaults.standard.set(value, forKey: key)
                        NSLog("registerDefaultsFromSettingsBundle: Set following to UserDefaults - (key: \(key), value: \(value), type: \(type(of: value)))")
                    }
                }
            }
        } else {
            NSLog("registerDefaultsFromSettingsBundle: Could not find Settings.bundle")
        }
    }
}

/// setDefaultsFromSettingsBundle() never overwrites a stored value, so installs that ran an
/// older build keep old defaults. Apply each revision of new defaults once.
func migrateHoleFillingDefaults() {
    let defaults = UserDefaults.standard
    let revisionKey = "HoleFillingDefaultsRevision"
    let revision = defaults.integer(forKey: revisionKey)
    if revision < 1 {
        defaults.set(1, forKey: "DepthConfidence")
        defaults.set(false, forKey: "CleanMesh")
        defaults.set(1, forKey: "TextureVertexColorPolicy")
        defaults.set(5.0, forKey: "MaxTextureDistance")
        defaults.set(4, forKey: "MaxDepth")
    }
    if revision < 2 {
        // Sharper textures and fewer surfaces seen by no keyframe: 2 keyframes per second,
        // full-resolution color images, higher on-device texture resolution, and up to
        // 4 texture atlases when exporting.
        defaults.set("2", forKey: "UpdateRate")
        defaults.set(true, forKey: "HDMode")
        defaults.set(2, forKey: "TextureResolution")
        defaults.set(4, forKey: "MaximumOutputTextures")
    }
    defaults.set(2, forKey: revisionKey)
}

@main
class AppDelegate: UIResponder, UIApplicationDelegate {

    var window: UIWindow?
    
    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?) -> Bool {
        
        // Always set Version to default
        let defaults = UserDefaults.standard
        defaults.removeObject(forKey: "Version")
        
        setDefaultsFromSettingsBundle()
        migrateHoleFillingDefaults()

        // Override point for customization after application launch.
        if !ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            // Ensure that the device supports scene depth and present
            //  an error-message view controller, if not.
            let storyboard = UIStoryboard(name: "Main", bundle: nil)
            window?.rootViewController = storyboard.instantiateViewController(withIdentifier: "unsupportedDeviceMessage")
        }
        return true
    }

    // MARK: UISceneSession Lifecycle

    func application(_ application: UIApplication, configurationForConnecting connectingSceneSession: UISceneSession, options: UIScene.ConnectionOptions) -> UISceneConfiguration {
        // Called when a new scene session is being created.
        // Use this method to select a configuration to create the new scene with.
        return UISceneConfiguration(name: "Default Configuration", sessionRole: connectingSceneSession.role)
    }

    func application(_ application: UIApplication, didDiscardSceneSessions sceneSessions: Set<UISceneSession>) {
        // Called when the user discards a scene session.
        // If any sessions were discarded while the application was not running, this will be called shortly after application:didFinishLaunchingWithOptions.
        // Use this method to release any resources that were specific to the discarded scenes, as they will not return.
    }
}

