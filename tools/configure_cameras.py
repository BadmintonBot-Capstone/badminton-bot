#!/usr/bin/env python3
"""Apply camera settings from config/camera.yaml to both Blackfly S cameras via PySpin."""

import yaml
import sys
import PySpin


def apply_settings(cam, cfg, role):
    """Apply exposure, gain, white balance, and frame rate to a single camera."""
    nodemap = cam.GetNodeMap()
    serial = cam.GetUniqueID()
    print(f"\n=== Configuring {role} camera ({serial}) ===")

    # Exposure.
    exposure_auto = PySpin.CEnumerationPtr(nodemap.GetNode("ExposureAuto"))
    exposure_auto.SetIntValue(exposure_auto.GetEntryByName("Off").GetValue())
    exposure_time = PySpin.CFloatPtr(nodemap.GetNode("ExposureTime"))
    exposure_time.SetValue(cfg["exposure_us"])
    print(f"  ExposureTime = {exposure_time.GetValue():.0f} µs")

    # Gain.
    gain_auto = PySpin.CEnumerationPtr(nodemap.GetNode("GainAuto"))
    gain_auto.SetIntValue(gain_auto.GetEntryByName("Off").GetValue())
    gain = PySpin.CFloatPtr(nodemap.GetNode("Gain"))
    gain.SetValue(cfg["gain_db"])
    print(f"  Gain = {gain.GetValue():.1f} dB")

    # White balance.
    wb_auto = PySpin.CEnumerationPtr(nodemap.GetNode("BalanceWhiteAuto"))
    mode = "Continuous" if cfg.get("white_balance_auto", False) else "Off"
    wb_auto.SetIntValue(wb_auto.GetEntryByName(mode).GetValue())
    print(f"  WhiteBalanceAuto = {mode}")

    # Frame rate.
    fr_enable = PySpin.CBooleanPtr(nodemap.GetNode("AcquisitionFrameRateEnable"))
    fr_enable.SetValue(True)
    fr = PySpin.CFloatPtr(nodemap.GetNode("AcquisitionFrameRate"))
    fr.SetValue(float(cfg["fps"]))
    print(f"  FrameRate = {fr.GetValue():.1f}")


def setup_trigger_primary(cam):
    """Enable 3.3V output on Line2."""
    nodemap = cam.GetNodeMap()
    line_sel = PySpin.CEnumerationPtr(nodemap.GetNode("LineSelector"))
    line_sel.SetIntValue(line_sel.GetEntryByName("Line2").GetValue())
    v33 = PySpin.CBooleanPtr(nodemap.GetNode("V3_3Enable"))
    v33.SetValue(True)
    print("  Trigger: primary (Line2 3.3V output enabled)")


def setup_trigger_secondary(cam):
    """Configure hardware trigger on Line3."""
    nodemap = cam.GetNodeMap()
    trig_mode = PySpin.CEnumerationPtr(nodemap.GetNode("TriggerMode"))
    trig_mode.SetIntValue(trig_mode.GetEntryByName("Off").GetValue())

    trig_src = PySpin.CEnumerationPtr(nodemap.GetNode("TriggerSource"))
    trig_src.SetIntValue(trig_src.GetEntryByName("Line3").GetValue())

    trig_overlap = PySpin.CEnumerationPtr(nodemap.GetNode("TriggerOverlap"))
    trig_overlap.SetIntValue(trig_overlap.GetEntryByName("ReadOut").GetValue())

    trig_mode.SetIntValue(trig_mode.GetEntryByName("On").GetValue())
    print("  Trigger: secondary (Line3 input, overlap=ReadOut)")


def main():
    config_path = sys.argv[1] if len(sys.argv) > 1 else "config/camera.yaml"
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()

    if cam_list.GetSize() < 2:
        print(f"Error: need 2 cameras, found {cam_list.GetSize()}")
        cam_list.Clear()
        system.ReleaseInstance()
        sys.exit(1)

    primary_serial = cfg.get("primary_serial", "")
    secondary_serial = cfg.get("secondary_serial", "")

    if primary_serial:
        primary = cam_list.GetBySerial(primary_serial)
        secondary = cam_list.GetBySerial(secondary_serial)
    else:
        primary = cam_list.GetByIndex(0)
        secondary = cam_list.GetByIndex(1)

    primary.Init()
    secondary.Init()

    apply_settings(primary, cfg, "primary")
    setup_trigger_primary(primary)

    apply_settings(secondary, cfg, "secondary")
    setup_trigger_secondary(secondary)

    primary.DeInit()
    secondary.DeInit()
    cam_list.Clear()
    system.ReleaseInstance()

    print("\nDone. Both cameras configured.")


if __name__ == "__main__":
    main()
