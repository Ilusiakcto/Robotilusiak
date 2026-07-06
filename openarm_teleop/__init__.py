"""OpenArm VR teleoperation.

Modules:
  kinematics    - Placo-based forward/inverse kinematics for the 7-DOF arm
  can_motor     - SocketCAN backend + Damiao MIT-mode motor protocol
  waveshare_can - Waveshare USB-CAN-FD-B backend (drop-in for can_motor.CANBus)
  teleop_udp    - VR-pose -> IK -> CAN control loop, via UDP (Quest APK)
  teleop        - VR-pose -> IK -> CAN control loop, via Adamo network (optional)
"""
