#!/usr/bin/env python3
"""
Webots Device Discovery Tool
Lists all devices on your robot to help configure the controller
"""

from controller import Robot

def main():
    robot = Robot()
    
    print("\n" + "=" * 60)
    print("WEBOTS DEVICE DISCOVERY")
    print("=" * 60)
    print(f"\nRobot Name: {robot.getName()}")
    print(f"\nTotal Devices: {robot.getNumberOfDevices()}")
    print("\nAvailable Devices:")
    print("-" * 60)
    
    motors = []
    cameras = []
    sensors = []
    others = []
    
    for i in range(robot.getNumberOfDevices()):
        device = robot.getDeviceByIndex(i)
        name = device.getName()
        device_type = device.getNodeType()
        
        print(f"{i:3d}. {name:30s} (Type: {device_type})")
        
        # Categorize
        if 'motor' in name.lower() or 'wheel' in name.lower():
            motors.append(name)
        elif 'camera' in name.lower() or 'cam' in name.lower():
            cameras.append(name)
        elif 'sensor' in name.lower():
            sensors.append(name)
        else:
            others.append(name)
    
    print("\n" + "=" * 60)
    print("CATEGORIZED DEVICES:")
    print("=" * 60)
    
    if motors:
        print("\n🔧 Motors/Wheels:")
        for m in motors:
            print(f"   - {m}")
    
    if cameras:
        print("\n📷 Cameras:")
        for c in cameras:
            print(f"   - {c}")
    
    if sensors:
        print("\n📡 Sensors:")
        for s in sensors:
            print(f"   - {s}")
    
    if others:
        print("\n❓ Other Devices:")
        for o in others:
            print(f"   - {o}")
    
    print("\n" + "=" * 60)
    print("CONFIGURATION HELP:")
    print("=" * 60)
    
    if len(motors) >= 2:
        print(f"\n✓ Found {len(motors)} potential motors")
        print("\nAdd these to motor_patterns in jetbot_ros2_controller.py:")
        if len(motors) >= 2:
            print(f"    ('{motors[0]}', '{motors[1]}'),")
    else:
        print("\n⚠ Need to find 2 motors for differential drive!")
    
    if cameras:
        print(f"\n✓ Found {len(cameras)} camera(s)")
        print("\nAdd these to camera_patterns in jetbot_ros2_controller.py:")
        for c in cameras:
            print(f"    '{c}',")
    
    print("\n" + "=" * 60)
    print("\nController will exit now. Use the device names above")
    print("to update jetbot_ros2_controller.py")
    print("=" * 60 + "\n")


if __name__ == '__main__':
    main()
