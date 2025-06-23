
"use strict";

let ApiOptions = require('./ApiOptions.js');
let KortexError = require('./KortexError.js');
let ErrorCodes = require('./ErrorCodes.js');
let SubErrorCodes = require('./SubErrorCodes.js');
let ActuatorConfig_SafetyLimitType = require('./ActuatorConfig_SafetyLimitType.js');
let AxisPosition = require('./AxisPosition.js');
let TorqueOffset = require('./TorqueOffset.js');
let ControlLoopParameters = require('./ControlLoopParameters.js');
let LoopSelection = require('./LoopSelection.js');
let CustomDataSelection = require('./CustomDataSelection.js');
let ControlLoop = require('./ControlLoop.js');
let ActuatorConfig_ControlMode = require('./ActuatorConfig_ControlMode.js');
let Servoing = require('./Servoing.js');
let ActuatorConfig_ControlModeInformation = require('./ActuatorConfig_ControlModeInformation.js');
let StepResponse = require('./StepResponse.js');
let FrequencyResponse = require('./FrequencyResponse.js');
let CoggingFeedforwardMode = require('./CoggingFeedforwardMode.js');
let AxisOffsets = require('./AxisOffsets.js');
let CustomDataIndex = require('./CustomDataIndex.js');
let EncoderDerivativeParameters = require('./EncoderDerivativeParameters.js');
let VectorDriveParameters = require('./VectorDriveParameters.js');
let ActuatorConfig_ServiceVersion = require('./ActuatorConfig_ServiceVersion.js');
let SafetyIdentifierBankA = require('./SafetyIdentifierBankA.js');
let CommandMode = require('./CommandMode.js');
let PositionCommand = require('./PositionCommand.js');
let TorqueCalibration = require('./TorqueCalibration.js');
let CoggingFeedforwardModeInformation = require('./CoggingFeedforwardModeInformation.js');
let RampResponse = require('./RampResponse.js');
let ControlLoopSelection = require('./ControlLoopSelection.js');
let CommandModeInformation = require('./CommandModeInformation.js');
let ActuatorCyclic_Feedback = require('./ActuatorCyclic_Feedback.js');
let CommandFlags = require('./CommandFlags.js');
let ActuatorCyclic_Command = require('./ActuatorCyclic_Command.js');
let ActuatorCyclic_CustomData = require('./ActuatorCyclic_CustomData.js');
let ActuatorCyclic_ServiceVersion = require('./ActuatorCyclic_ServiceVersion.js');
let StatusFlags = require('./StatusFlags.js');
let ActuatorCyclic_MessageId = require('./ActuatorCyclic_MessageId.js');
let SystemTime = require('./SystemTime.js');
let UserNotification = require('./UserNotification.js');
let CartesianSpeed = require('./CartesianSpeed.js');
let TrajectoryErrorType = require('./TrajectoryErrorType.js');
let SequenceTasksRange = require('./SequenceTasksRange.js');
let ActionList = require('./ActionList.js');
let WifiEncryptionType = require('./WifiEncryptionType.js');
let Point = require('./Point.js');
let ControllerList = require('./ControllerList.js');
let BridgeResult = require('./BridgeResult.js');
let Base_RotationMatrix = require('./Base_RotationMatrix.js');
let ControllerNotification = require('./ControllerNotification.js');
let ShapeType = require('./ShapeType.js');
let TransformationMatrix = require('./TransformationMatrix.js');
let TrajectoryErrorElement = require('./TrajectoryErrorElement.js');
let NetworkNotification = require('./NetworkNotification.js');
let BridgeStatus = require('./BridgeStatus.js');
let Xbox360AnalogInputIdentifier = require('./Xbox360AnalogInputIdentifier.js');
let CommunicationInterfaceConfiguration = require('./CommunicationInterfaceConfiguration.js');
let ActionHandle = require('./ActionHandle.js');
let JointTorque = require('./JointTorque.js');
let GripperCommand = require('./GripperCommand.js');
let WaypointList = require('./WaypointList.js');
let Base_ControlModeNotification = require('./Base_ControlModeNotification.js');
let BridgePortConfig = require('./BridgePortConfig.js');
let Xbox360DigitalInputIdentifier = require('./Xbox360DigitalInputIdentifier.js');
let CartesianLimitation = require('./CartesianLimitation.js');
let MapHandle = require('./MapHandle.js');
let ProtectionZoneNotification = require('./ProtectionZoneNotification.js');
let ArmStateInformation = require('./ArmStateInformation.js');
let EventIdSequenceInfoNotification = require('./EventIdSequenceInfoNotification.js');
let ServoingModeNotification = require('./ServoingModeNotification.js');
let ConfigurationChangeNotificationList = require('./ConfigurationChangeNotificationList.js');
let Base_CapSenseConfig = require('./Base_CapSenseConfig.js');
let ControllerConfigurationMode = require('./ControllerConfigurationMode.js');
let Waypoint = require('./Waypoint.js');
let EmergencyStop = require('./EmergencyStop.js');
let ControllerEvent = require('./ControllerEvent.js');
let ProtectionZoneEvent = require('./ProtectionZoneEvent.js');
let TrajectoryContinuityMode = require('./TrajectoryContinuityMode.js');
let WifiInformation = require('./WifiInformation.js');
let ActionType = require('./ActionType.js');
let CartesianLimitationList = require('./CartesianLimitationList.js');
let ControllerNotification_state = require('./ControllerNotification_state.js');
let ControllerHandle = require('./ControllerHandle.js');
let Mapping = require('./Mapping.js');
let Action_action_parameters = require('./Action_action_parameters.js');
let ControllerElementHandle = require('./ControllerElementHandle.js');
let RobotEventNotificationList = require('./RobotEventNotificationList.js');
let IPv4Information = require('./IPv4Information.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');
let RobotEvent = require('./RobotEvent.js');
let JointTorques = require('./JointTorques.js');
let ConfigurationNotificationEvent = require('./ConfigurationNotificationEvent.js');
let Wrench = require('./Wrench.js');
let SequenceInformation = require('./SequenceInformation.js');
let SequenceTask = require('./SequenceTask.js');
let AppendActionInformation = require('./AppendActionInformation.js');
let MappingHandle = require('./MappingHandle.js');
let RFConfiguration = require('./RFConfiguration.js');
let Base_SafetyIdentifier = require('./Base_SafetyIdentifier.js');
let GripperRequest = require('./GripperRequest.js');
let Delay = require('./Delay.js');
let Base_CapSenseMode = require('./Base_CapSenseMode.js');
let ControllerState = require('./ControllerState.js');
let Faults = require('./Faults.js');
let WaypointValidationReport = require('./WaypointValidationReport.js');
let MapElement = require('./MapElement.js');
let SafetyEvent = require('./SafetyEvent.js');
let ConstrainedJointAngles = require('./ConstrainedJointAngles.js');
let Snapshot = require('./Snapshot.js');
let OperatingModeNotificationList = require('./OperatingModeNotificationList.js');
let JointTrajectoryConstraint = require('./JointTrajectoryConstraint.js');
let ZoneShape = require('./ZoneShape.js');
let SequenceTasks = require('./SequenceTasks.js');
let SequenceList = require('./SequenceList.js');
let BridgeConfig = require('./BridgeConfig.js');
let GpioCommand = require('./GpioCommand.js');
let ControllerEventType = require('./ControllerEventType.js');
let Query = require('./Query.js');
let UserProfileList = require('./UserProfileList.js');
let ConfigurationChangeNotification = require('./ConfigurationChangeNotification.js');
let AdmittanceMode = require('./AdmittanceMode.js');
let SequenceHandle = require('./SequenceHandle.js');
let ProtectionZoneInformation = require('./ProtectionZoneInformation.js');
let WifiSecurityType = require('./WifiSecurityType.js');
let UserEvent = require('./UserEvent.js');
let UserNotificationList = require('./UserNotificationList.js');
let UserList = require('./UserList.js');
let Base_ServiceVersion = require('./Base_ServiceVersion.js');
let ControllerInputType = require('./ControllerInputType.js');
let Orientation = require('./Orientation.js');
let AngularWaypoint = require('./AngularWaypoint.js');
let ServoingMode = require('./ServoingMode.js');
let Base_Stop = require('./Base_Stop.js');
let OperatingModeNotification = require('./OperatingModeNotification.js');
let SequenceTaskHandle = require('./SequenceTaskHandle.js');
let ConfigurationChangeNotification_configuration_change = require('./ConfigurationChangeNotification_configuration_change.js');
let NavigationDirection = require('./NavigationDirection.js');
let NetworkNotificationList = require('./NetworkNotificationList.js');
let MapGroup = require('./MapGroup.js');
let Gripper = require('./Gripper.js');
let JointsLimitationsList = require('./JointsLimitationsList.js');
let GripperMode = require('./GripperMode.js');
let ConstrainedPosition = require('./ConstrainedPosition.js');
let NetworkType = require('./NetworkType.js');
let JointAngle = require('./JointAngle.js');
let PreComputedJointTrajectory = require('./PreComputedJointTrajectory.js');
let WifiEnableState = require('./WifiEnableState.js');
let Waypoint_type_of_waypoint = require('./Waypoint_type_of_waypoint.js');
let Twist = require('./Twist.js');
let TrajectoryInfoType = require('./TrajectoryInfoType.js');
let IPv4Configuration = require('./IPv4Configuration.js');
let SignalQuality = require('./SignalQuality.js');
let RequestedActionType = require('./RequestedActionType.js');
let ProtectionZone = require('./ProtectionZone.js');
let ChangeJointSpeeds = require('./ChangeJointSpeeds.js');
let Admittance = require('./Admittance.js');
let GpioBehavior = require('./GpioBehavior.js');
let CartesianTrajectoryConstraint_type = require('./CartesianTrajectoryConstraint_type.js');
let ConstrainedOrientation = require('./ConstrainedOrientation.js');
let NetworkEvent = require('./NetworkEvent.js');
let ConstrainedJointAngle = require('./ConstrainedJointAngle.js');
let ChangeWrench = require('./ChangeWrench.js');
let UserProfile = require('./UserProfile.js');
let GpioPinPropertyFlags = require('./GpioPinPropertyFlags.js');
let ControllerType = require('./ControllerType.js');
let ControllerElementState = require('./ControllerElementState.js');
let OperatingMode = require('./OperatingMode.js');
let JointLimitation = require('./JointLimitation.js');
let JointAngles = require('./JointAngles.js');
let SafetyNotificationList = require('./SafetyNotificationList.js');
let SequenceInfoNotificationList = require('./SequenceInfoNotificationList.js');
let MapEvent = require('./MapEvent.js');
let SequenceTaskConfiguration = require('./SequenceTaskConfiguration.js');
let MapEvent_events = require('./MapEvent_events.js');
let GpioEvent = require('./GpioEvent.js');
let SequenceTasksPair = require('./SequenceTasksPair.js');
let Base_JointSpeeds = require('./Base_JointSpeeds.js');
let BridgeList = require('./BridgeList.js');
let ServoingModeInformation = require('./ServoingModeInformation.js');
let WrenchMode = require('./WrenchMode.js');
let ActionNotification = require('./ActionNotification.js');
let ActuatorInformation = require('./ActuatorInformation.js');
let Action = require('./Action.js');
let Ssid = require('./Ssid.js');
let Timeout = require('./Timeout.js');
let TwistLimitation = require('./TwistLimitation.js');
let ControllerNotificationList = require('./ControllerNotificationList.js');
let IKData = require('./IKData.js');
let FactoryNotification = require('./FactoryNotification.js');
let ActivateMapHandle = require('./ActivateMapHandle.js');
let TwistCommand = require('./TwistCommand.js');
let WristDigitalInputIdentifier = require('./WristDigitalInputIdentifier.js');
let RobotEventNotification = require('./RobotEventNotification.js');
let Base_Position = require('./Base_Position.js');
let Base_ControlModeInformation = require('./Base_ControlModeInformation.js');
let PasswordChange = require('./PasswordChange.js');
let Pose = require('./Pose.js');
let SnapshotType = require('./SnapshotType.js');
let PreComputedJointTrajectoryElement = require('./PreComputedJointTrajectoryElement.js');
let WifiInformationList = require('./WifiInformationList.js');
let Base_GpioConfiguration = require('./Base_GpioConfiguration.js');
let ServoingModeNotificationList = require('./ServoingModeNotificationList.js');
let GpioConfigurationList = require('./GpioConfigurationList.js');
let SoundType = require('./SoundType.js');
let TrajectoryErrorIdentifier = require('./TrajectoryErrorIdentifier.js');
let MappingInfoNotification = require('./MappingInfoNotification.js');
let AdvancedSequenceHandle = require('./AdvancedSequenceHandle.js');
let ActionEvent = require('./ActionEvent.js');
let LedState = require('./LedState.js');
let SequenceTasksConfiguration = require('./SequenceTasksConfiguration.js');
let ArmStateNotification = require('./ArmStateNotification.js');
let FirmwareComponentVersion = require('./FirmwareComponentVersion.js');
let ActionExecutionState = require('./ActionExecutionState.js');
let ConstrainedPose = require('./ConstrainedPose.js');
let ActionNotificationList = require('./ActionNotificationList.js');
let SwitchControlMapping = require('./SwitchControlMapping.js');
let WifiConfiguration = require('./WifiConfiguration.js');
let SequenceInfoNotification = require('./SequenceInfoNotification.js');
let GpioPinConfiguration = require('./GpioPinConfiguration.js');
let MapList = require('./MapList.js');
let ControllerConfigurationList = require('./ControllerConfigurationList.js');
let MappingInfoNotificationList = require('./MappingInfoNotificationList.js');
let FactoryEvent = require('./FactoryEvent.js');
let WifiConfigurationList = require('./WifiConfigurationList.js');
let FirmwareBundleVersions = require('./FirmwareBundleVersions.js');
let KinematicTrajectoryConstraints = require('./KinematicTrajectoryConstraints.js');
let BackupEvent = require('./BackupEvent.js');
let WrenchLimitation = require('./WrenchLimitation.js');
let JointSpeed = require('./JointSpeed.js');
let BridgeIdentifier = require('./BridgeIdentifier.js');
let ControlModeNotificationList = require('./ControlModeNotificationList.js');
let OperatingModeInformation = require('./OperatingModeInformation.js');
let BluetoothEnableState = require('./BluetoothEnableState.js');
let GpioAction = require('./GpioAction.js');
let ProtectionZoneList = require('./ProtectionZoneList.js');
let Gen3GpioPinId = require('./Gen3GpioPinId.js');
let CartesianWaypoint = require('./CartesianWaypoint.js');
let MapGroupList = require('./MapGroupList.js');
let TrajectoryErrorReport = require('./TrajectoryErrorReport.js');
let FullIPv4Configuration = require('./FullIPv4Configuration.js');
let ProtectionZoneNotificationList = require('./ProtectionZoneNotificationList.js');
let FullUserProfile = require('./FullUserProfile.js');
let TrajectoryInfo = require('./TrajectoryInfo.js');
let NetworkHandle = require('./NetworkHandle.js');
let Sequence = require('./Sequence.js');
let MapGroupHandle = require('./MapGroupHandle.js');
let WrenchCommand = require('./WrenchCommand.js');
let Base_RotationMatrixRow = require('./Base_RotationMatrixRow.js');
let BridgeType = require('./BridgeType.js');
let TransformationRow = require('./TransformationRow.js');
let ControllerElementEventType = require('./ControllerElementEventType.js');
let MappingList = require('./MappingList.js');
let JointTrajectoryConstraintType = require('./JointTrajectoryConstraintType.js');
let JointNavigationDirection = require('./JointNavigationDirection.js');
let Finger = require('./Finger.js');
let ControllerElementHandle_identifier = require('./ControllerElementHandle_identifier.js');
let Base_ControlMode = require('./Base_ControlMode.js');
let Map = require('./Map.js');
let ControllerConfiguration = require('./ControllerConfiguration.js');
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let ChangeTwist = require('./ChangeTwist.js');
let ControllerBehavior = require('./ControllerBehavior.js');
let LimitationType = require('./LimitationType.js');
let BaseFeedback = require('./BaseFeedback.js');
let ActuatorCustomData = require('./ActuatorCustomData.js');
let BaseCyclic_CustomData = require('./BaseCyclic_CustomData.js');
let ActuatorFeedback = require('./ActuatorFeedback.js');
let BaseCyclic_ServiceVersion = require('./BaseCyclic_ServiceVersion.js');
let ActuatorCommand = require('./ActuatorCommand.js');
let BaseCyclic_Command = require('./BaseCyclic_Command.js');
let BaseCyclic_Feedback = require('./BaseCyclic_Feedback.js');
let NotificationHandle = require('./NotificationHandle.js');
let Empty = require('./Empty.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let UARTDeviceIdentification = require('./UARTDeviceIdentification.js');
let Connection = require('./Connection.js');
let CartesianReferenceFrame = require('./CartesianReferenceFrame.js');
let ArmState = require('./ArmState.js');
let NotificationType = require('./NotificationType.js');
let DeviceTypes = require('./DeviceTypes.js');
let UARTStopBits = require('./UARTStopBits.js');
let SafetyHandle = require('./SafetyHandle.js');
let UARTParity = require('./UARTParity.js');
let Unit = require('./Unit.js');
let SafetyNotification = require('./SafetyNotification.js');
let Timestamp = require('./Timestamp.js');
let SafetyStatusValue = require('./SafetyStatusValue.js');
let UARTConfiguration = require('./UARTConfiguration.js');
let CountryCodeIdentifier = require('./CountryCodeIdentifier.js');
let NotificationOptions = require('./NotificationOptions.js');
let UARTSpeed = require('./UARTSpeed.js');
let Permission = require('./Permission.js');
let UARTWordLength = require('./UARTWordLength.js');
let DeviceHandle = require('./DeviceHandle.js');
let CountryCode = require('./CountryCode.js');
let CartesianTransform = require('./CartesianTransform.js');
let CartesianReferenceFrameInfo = require('./CartesianReferenceFrameInfo.js');
let TwistLinearSoftLimit = require('./TwistLinearSoftLimit.js');
let ControlConfigurationEvent = require('./ControlConfigurationEvent.js');
let ControlConfig_ControlMode = require('./ControlConfig_ControlMode.js');
let ControlConfig_ControlModeNotification = require('./ControlConfig_ControlModeNotification.js');
let DesiredSpeeds = require('./DesiredSpeeds.js');
let JointAccelerationSoftLimits = require('./JointAccelerationSoftLimits.js');
let KinematicLimitsList = require('./KinematicLimitsList.js');
let GravityVector = require('./GravityVector.js');
let ToolConfiguration = require('./ToolConfiguration.js');
let ControlConfig_ServiceVersion = require('./ControlConfig_ServiceVersion.js');
let TwistAngularSoftLimit = require('./TwistAngularSoftLimit.js');
let ControlConfig_ControlModeInformation = require('./ControlConfig_ControlModeInformation.js');
let ControlConfigurationNotification = require('./ControlConfigurationNotification.js');
let ControlConfig_JointSpeeds = require('./ControlConfig_JointSpeeds.js');
let AngularTwist = require('./AngularTwist.js');
let ControlConfig_Position = require('./ControlConfig_Position.js');
let LinearTwist = require('./LinearTwist.js');
let JointSpeedSoftLimits = require('./JointSpeedSoftLimits.js');
let PayloadInformation = require('./PayloadInformation.js');
let KinematicLimits = require('./KinematicLimits.js');
let SafetyThreshold = require('./SafetyThreshold.js');
let SerialNumber = require('./SerialNumber.js');
let PowerOnSelfTestResult = require('./PowerOnSelfTestResult.js');
let DeviceConfig_ServiceVersion = require('./DeviceConfig_ServiceVersion.js');
let SafetyEnable = require('./SafetyEnable.js');
let PartNumber = require('./PartNumber.js');
let PartNumberRevision = require('./PartNumberRevision.js');
let DeviceType = require('./DeviceType.js');
let CalibrationStatus = require('./CalibrationStatus.js');
let DeviceConfig_SafetyLimitType = require('./DeviceConfig_SafetyLimitType.js');
let CalibrationResult = require('./CalibrationResult.js');
let CalibrationItem = require('./CalibrationItem.js');
let CalibrationParameter = require('./CalibrationParameter.js');
let DeviceConfig_CapSenseMode = require('./DeviceConfig_CapSenseMode.js');
let RunMode = require('./RunMode.js');
let SafetyInformationList = require('./SafetyInformationList.js');
let MACAddress = require('./MACAddress.js');
let FirmwareVersion = require('./FirmwareVersion.js');
let DeviceConfig_CapSenseConfig = require('./DeviceConfig_CapSenseConfig.js');
let ModelNumber = require('./ModelNumber.js');
let SafetyInformation = require('./SafetyInformation.js');
let CalibrationParameter_value = require('./CalibrationParameter_value.js');
let CalibrationElement = require('./CalibrationElement.js');
let SafetyConfigurationList = require('./SafetyConfigurationList.js');
let CapSenseRegister = require('./CapSenseRegister.js');
let RunModes = require('./RunModes.js');
let IPv4Settings = require('./IPv4Settings.js');
let SafetyConfiguration = require('./SafetyConfiguration.js');
let Calibration = require('./Calibration.js');
let SafetyStatus = require('./SafetyStatus.js');
let BootloaderVersion = require('./BootloaderVersion.js');
let RebootRqst = require('./RebootRqst.js');
let DeviceManager_ServiceVersion = require('./DeviceManager_ServiceVersion.js');
let DeviceHandles = require('./DeviceHandles.js');
let GripperConfig_SafetyIdentifier = require('./GripperConfig_SafetyIdentifier.js');
let RobotiqGripperStatusFlags = require('./RobotiqGripperStatusFlags.js');
let MotorCommand = require('./MotorCommand.js');
let GripperCyclic_MessageId = require('./GripperCyclic_MessageId.js');
let GripperCyclic_Feedback = require('./GripperCyclic_Feedback.js');
let CustomDataUnit = require('./CustomDataUnit.js');
let GripperCyclic_CustomData = require('./GripperCyclic_CustomData.js');
let GripperCyclic_ServiceVersion = require('./GripperCyclic_ServiceVersion.js');
let MotorFeedback = require('./MotorFeedback.js');
let GripperCyclic_Command = require('./GripperCyclic_Command.js');
let I2CWriteParameter = require('./I2CWriteParameter.js');
let I2CRegisterAddressSize = require('./I2CRegisterAddressSize.js');
let I2CDeviceIdentification = require('./I2CDeviceIdentification.js');
let GPIOState = require('./GPIOState.js');
let GPIOPull = require('./GPIOPull.js');
let I2CMode = require('./I2CMode.js');
let InterconnectConfig_SafetyIdentifier = require('./InterconnectConfig_SafetyIdentifier.js');
let I2CWriteRegisterParameter = require('./I2CWriteRegisterParameter.js');
let UARTPortId = require('./UARTPortId.js');
let I2CConfiguration = require('./I2CConfiguration.js');
let EthernetSpeed = require('./EthernetSpeed.js');
let EthernetDuplex = require('./EthernetDuplex.js');
let InterconnectConfig_ServiceVersion = require('./InterconnectConfig_ServiceVersion.js');
let InterconnectConfig_GPIOConfiguration = require('./InterconnectConfig_GPIOConfiguration.js');
let EthernetConfiguration = require('./EthernetConfiguration.js');
let I2CData = require('./I2CData.js');
let GPIOMode = require('./GPIOMode.js');
let I2CDevice = require('./I2CDevice.js');
let I2CReadParameter = require('./I2CReadParameter.js');
let GPIOIdentifier = require('./GPIOIdentifier.js');
let GPIOValue = require('./GPIOValue.js');
let I2CReadRegisterParameter = require('./I2CReadRegisterParameter.js');
let EthernetDeviceIdentification = require('./EthernetDeviceIdentification.js');
let I2CDeviceAddressing = require('./I2CDeviceAddressing.js');
let GPIOIdentification = require('./GPIOIdentification.js');
let EthernetDevice = require('./EthernetDevice.js');
let InterconnectCyclic_Feedback = require('./InterconnectCyclic_Feedback.js');
let InterconnectCyclic_MessageId = require('./InterconnectCyclic_MessageId.js');
let InterconnectCyclic_Feedback_tool_feedback = require('./InterconnectCyclic_Feedback_tool_feedback.js');
let InterconnectCyclic_Command_tool_command = require('./InterconnectCyclic_Command_tool_command.js');
let InterconnectCyclic_CustomData_tool_customData = require('./InterconnectCyclic_CustomData_tool_customData.js');
let InterconnectCyclic_CustomData = require('./InterconnectCyclic_CustomData.js');
let InterconnectCyclic_Command = require('./InterconnectCyclic_Command.js');
let InterconnectCyclic_ServiceVersion = require('./InterconnectCyclic_ServiceVersion.js');
let BaseType = require('./BaseType.js');
let CompleteProductConfiguration = require('./CompleteProductConfiguration.js');
let BrakeType = require('./BrakeType.js');
let ProductConfigurationEndEffectorType = require('./ProductConfigurationEndEffectorType.js');
let WristType = require('./WristType.js');
let ArmLaterality = require('./ArmLaterality.js');
let VisionModuleType = require('./VisionModuleType.js');
let InterfaceModuleType = require('./InterfaceModuleType.js');
let EndEffectorType = require('./EndEffectorType.js');
let ModelId = require('./ModelId.js');
let FocusPoint = require('./FocusPoint.js');
let Resolution = require('./Resolution.js');
let VisionEvent = require('./VisionEvent.js');
let Option = require('./Option.js');
let FrameRate = require('./FrameRate.js');
let VisionConfig_RotationMatrix = require('./VisionConfig_RotationMatrix.js');
let SensorFocusAction = require('./SensorFocusAction.js');
let OptionValue = require('./OptionValue.js');
let OptionIdentifier = require('./OptionIdentifier.js');
let ManualFocus = require('./ManualFocus.js');
let TranslationVector = require('./TranslationVector.js');
let IntrinsicProfileIdentifier = require('./IntrinsicProfileIdentifier.js');
let VisionConfig_RotationMatrixRow = require('./VisionConfig_RotationMatrixRow.js');
let SensorIdentifier = require('./SensorIdentifier.js');
let BitRate = require('./BitRate.js');
let Sensor = require('./Sensor.js');
let DistortionCoefficients = require('./DistortionCoefficients.js');
let ExtrinsicParameters = require('./ExtrinsicParameters.js');
let SensorFocusAction_action_parameters = require('./SensorFocusAction_action_parameters.js');
let SensorSettings = require('./SensorSettings.js');
let VisionNotification = require('./VisionNotification.js');
let VisionConfig_ServiceVersion = require('./VisionConfig_ServiceVersion.js');
let IntrinsicParameters = require('./IntrinsicParameters.js');
let OptionInformation = require('./OptionInformation.js');
let FocusAction = require('./FocusAction.js');
let FollowCartesianTrajectoryActionGoal = require('./FollowCartesianTrajectoryActionGoal.js');
let FollowCartesianTrajectoryGoal = require('./FollowCartesianTrajectoryGoal.js');
let FollowCartesianTrajectoryResult = require('./FollowCartesianTrajectoryResult.js');
let FollowCartesianTrajectoryAction = require('./FollowCartesianTrajectoryAction.js');
let FollowCartesianTrajectoryActionFeedback = require('./FollowCartesianTrajectoryActionFeedback.js');
let FollowCartesianTrajectoryActionResult = require('./FollowCartesianTrajectoryActionResult.js');
let FollowCartesianTrajectoryFeedback = require('./FollowCartesianTrajectoryFeedback.js');

module.exports = {
  ApiOptions: ApiOptions,
  KortexError: KortexError,
  ErrorCodes: ErrorCodes,
  SubErrorCodes: SubErrorCodes,
  ActuatorConfig_SafetyLimitType: ActuatorConfig_SafetyLimitType,
  AxisPosition: AxisPosition,
  TorqueOffset: TorqueOffset,
  ControlLoopParameters: ControlLoopParameters,
  LoopSelection: LoopSelection,
  CustomDataSelection: CustomDataSelection,
  ControlLoop: ControlLoop,
  ActuatorConfig_ControlMode: ActuatorConfig_ControlMode,
  Servoing: Servoing,
  ActuatorConfig_ControlModeInformation: ActuatorConfig_ControlModeInformation,
  StepResponse: StepResponse,
  FrequencyResponse: FrequencyResponse,
  CoggingFeedforwardMode: CoggingFeedforwardMode,
  AxisOffsets: AxisOffsets,
  CustomDataIndex: CustomDataIndex,
  EncoderDerivativeParameters: EncoderDerivativeParameters,
  VectorDriveParameters: VectorDriveParameters,
  ActuatorConfig_ServiceVersion: ActuatorConfig_ServiceVersion,
  SafetyIdentifierBankA: SafetyIdentifierBankA,
  CommandMode: CommandMode,
  PositionCommand: PositionCommand,
  TorqueCalibration: TorqueCalibration,
  CoggingFeedforwardModeInformation: CoggingFeedforwardModeInformation,
  RampResponse: RampResponse,
  ControlLoopSelection: ControlLoopSelection,
  CommandModeInformation: CommandModeInformation,
  ActuatorCyclic_Feedback: ActuatorCyclic_Feedback,
  CommandFlags: CommandFlags,
  ActuatorCyclic_Command: ActuatorCyclic_Command,
  ActuatorCyclic_CustomData: ActuatorCyclic_CustomData,
  ActuatorCyclic_ServiceVersion: ActuatorCyclic_ServiceVersion,
  StatusFlags: StatusFlags,
  ActuatorCyclic_MessageId: ActuatorCyclic_MessageId,
  SystemTime: SystemTime,
  UserNotification: UserNotification,
  CartesianSpeed: CartesianSpeed,
  TrajectoryErrorType: TrajectoryErrorType,
  SequenceTasksRange: SequenceTasksRange,
  ActionList: ActionList,
  WifiEncryptionType: WifiEncryptionType,
  Point: Point,
  ControllerList: ControllerList,
  BridgeResult: BridgeResult,
  Base_RotationMatrix: Base_RotationMatrix,
  ControllerNotification: ControllerNotification,
  ShapeType: ShapeType,
  TransformationMatrix: TransformationMatrix,
  TrajectoryErrorElement: TrajectoryErrorElement,
  NetworkNotification: NetworkNotification,
  BridgeStatus: BridgeStatus,
  Xbox360AnalogInputIdentifier: Xbox360AnalogInputIdentifier,
  CommunicationInterfaceConfiguration: CommunicationInterfaceConfiguration,
  ActionHandle: ActionHandle,
  JointTorque: JointTorque,
  GripperCommand: GripperCommand,
  WaypointList: WaypointList,
  Base_ControlModeNotification: Base_ControlModeNotification,
  BridgePortConfig: BridgePortConfig,
  Xbox360DigitalInputIdentifier: Xbox360DigitalInputIdentifier,
  CartesianLimitation: CartesianLimitation,
  MapHandle: MapHandle,
  ProtectionZoneNotification: ProtectionZoneNotification,
  ArmStateInformation: ArmStateInformation,
  EventIdSequenceInfoNotification: EventIdSequenceInfoNotification,
  ServoingModeNotification: ServoingModeNotification,
  ConfigurationChangeNotificationList: ConfigurationChangeNotificationList,
  Base_CapSenseConfig: Base_CapSenseConfig,
  ControllerConfigurationMode: ControllerConfigurationMode,
  Waypoint: Waypoint,
  EmergencyStop: EmergencyStop,
  ControllerEvent: ControllerEvent,
  ProtectionZoneEvent: ProtectionZoneEvent,
  TrajectoryContinuityMode: TrajectoryContinuityMode,
  WifiInformation: WifiInformation,
  ActionType: ActionType,
  CartesianLimitationList: CartesianLimitationList,
  ControllerNotification_state: ControllerNotification_state,
  ControllerHandle: ControllerHandle,
  Mapping: Mapping,
  Action_action_parameters: Action_action_parameters,
  ControllerElementHandle: ControllerElementHandle,
  RobotEventNotificationList: RobotEventNotificationList,
  IPv4Information: IPv4Information,
  CartesianTrajectoryConstraint: CartesianTrajectoryConstraint,
  RobotEvent: RobotEvent,
  JointTorques: JointTorques,
  ConfigurationNotificationEvent: ConfigurationNotificationEvent,
  Wrench: Wrench,
  SequenceInformation: SequenceInformation,
  SequenceTask: SequenceTask,
  AppendActionInformation: AppendActionInformation,
  MappingHandle: MappingHandle,
  RFConfiguration: RFConfiguration,
  Base_SafetyIdentifier: Base_SafetyIdentifier,
  GripperRequest: GripperRequest,
  Delay: Delay,
  Base_CapSenseMode: Base_CapSenseMode,
  ControllerState: ControllerState,
  Faults: Faults,
  WaypointValidationReport: WaypointValidationReport,
  MapElement: MapElement,
  SafetyEvent: SafetyEvent,
  ConstrainedJointAngles: ConstrainedJointAngles,
  Snapshot: Snapshot,
  OperatingModeNotificationList: OperatingModeNotificationList,
  JointTrajectoryConstraint: JointTrajectoryConstraint,
  ZoneShape: ZoneShape,
  SequenceTasks: SequenceTasks,
  SequenceList: SequenceList,
  BridgeConfig: BridgeConfig,
  GpioCommand: GpioCommand,
  ControllerEventType: ControllerEventType,
  Query: Query,
  UserProfileList: UserProfileList,
  ConfigurationChangeNotification: ConfigurationChangeNotification,
  AdmittanceMode: AdmittanceMode,
  SequenceHandle: SequenceHandle,
  ProtectionZoneInformation: ProtectionZoneInformation,
  WifiSecurityType: WifiSecurityType,
  UserEvent: UserEvent,
  UserNotificationList: UserNotificationList,
  UserList: UserList,
  Base_ServiceVersion: Base_ServiceVersion,
  ControllerInputType: ControllerInputType,
  Orientation: Orientation,
  AngularWaypoint: AngularWaypoint,
  ServoingMode: ServoingMode,
  Base_Stop: Base_Stop,
  OperatingModeNotification: OperatingModeNotification,
  SequenceTaskHandle: SequenceTaskHandle,
  ConfigurationChangeNotification_configuration_change: ConfigurationChangeNotification_configuration_change,
  NavigationDirection: NavigationDirection,
  NetworkNotificationList: NetworkNotificationList,
  MapGroup: MapGroup,
  Gripper: Gripper,
  JointsLimitationsList: JointsLimitationsList,
  GripperMode: GripperMode,
  ConstrainedPosition: ConstrainedPosition,
  NetworkType: NetworkType,
  JointAngle: JointAngle,
  PreComputedJointTrajectory: PreComputedJointTrajectory,
  WifiEnableState: WifiEnableState,
  Waypoint_type_of_waypoint: Waypoint_type_of_waypoint,
  Twist: Twist,
  TrajectoryInfoType: TrajectoryInfoType,
  IPv4Configuration: IPv4Configuration,
  SignalQuality: SignalQuality,
  RequestedActionType: RequestedActionType,
  ProtectionZone: ProtectionZone,
  ChangeJointSpeeds: ChangeJointSpeeds,
  Admittance: Admittance,
  GpioBehavior: GpioBehavior,
  CartesianTrajectoryConstraint_type: CartesianTrajectoryConstraint_type,
  ConstrainedOrientation: ConstrainedOrientation,
  NetworkEvent: NetworkEvent,
  ConstrainedJointAngle: ConstrainedJointAngle,
  ChangeWrench: ChangeWrench,
  UserProfile: UserProfile,
  GpioPinPropertyFlags: GpioPinPropertyFlags,
  ControllerType: ControllerType,
  ControllerElementState: ControllerElementState,
  OperatingMode: OperatingMode,
  JointLimitation: JointLimitation,
  JointAngles: JointAngles,
  SafetyNotificationList: SafetyNotificationList,
  SequenceInfoNotificationList: SequenceInfoNotificationList,
  MapEvent: MapEvent,
  SequenceTaskConfiguration: SequenceTaskConfiguration,
  MapEvent_events: MapEvent_events,
  GpioEvent: GpioEvent,
  SequenceTasksPair: SequenceTasksPair,
  Base_JointSpeeds: Base_JointSpeeds,
  BridgeList: BridgeList,
  ServoingModeInformation: ServoingModeInformation,
  WrenchMode: WrenchMode,
  ActionNotification: ActionNotification,
  ActuatorInformation: ActuatorInformation,
  Action: Action,
  Ssid: Ssid,
  Timeout: Timeout,
  TwistLimitation: TwistLimitation,
  ControllerNotificationList: ControllerNotificationList,
  IKData: IKData,
  FactoryNotification: FactoryNotification,
  ActivateMapHandle: ActivateMapHandle,
  TwistCommand: TwistCommand,
  WristDigitalInputIdentifier: WristDigitalInputIdentifier,
  RobotEventNotification: RobotEventNotification,
  Base_Position: Base_Position,
  Base_ControlModeInformation: Base_ControlModeInformation,
  PasswordChange: PasswordChange,
  Pose: Pose,
  SnapshotType: SnapshotType,
  PreComputedJointTrajectoryElement: PreComputedJointTrajectoryElement,
  WifiInformationList: WifiInformationList,
  Base_GpioConfiguration: Base_GpioConfiguration,
  ServoingModeNotificationList: ServoingModeNotificationList,
  GpioConfigurationList: GpioConfigurationList,
  SoundType: SoundType,
  TrajectoryErrorIdentifier: TrajectoryErrorIdentifier,
  MappingInfoNotification: MappingInfoNotification,
  AdvancedSequenceHandle: AdvancedSequenceHandle,
  ActionEvent: ActionEvent,
  LedState: LedState,
  SequenceTasksConfiguration: SequenceTasksConfiguration,
  ArmStateNotification: ArmStateNotification,
  FirmwareComponentVersion: FirmwareComponentVersion,
  ActionExecutionState: ActionExecutionState,
  ConstrainedPose: ConstrainedPose,
  ActionNotificationList: ActionNotificationList,
  SwitchControlMapping: SwitchControlMapping,
  WifiConfiguration: WifiConfiguration,
  SequenceInfoNotification: SequenceInfoNotification,
  GpioPinConfiguration: GpioPinConfiguration,
  MapList: MapList,
  ControllerConfigurationList: ControllerConfigurationList,
  MappingInfoNotificationList: MappingInfoNotificationList,
  FactoryEvent: FactoryEvent,
  WifiConfigurationList: WifiConfigurationList,
  FirmwareBundleVersions: FirmwareBundleVersions,
  KinematicTrajectoryConstraints: KinematicTrajectoryConstraints,
  BackupEvent: BackupEvent,
  WrenchLimitation: WrenchLimitation,
  JointSpeed: JointSpeed,
  BridgeIdentifier: BridgeIdentifier,
  ControlModeNotificationList: ControlModeNotificationList,
  OperatingModeInformation: OperatingModeInformation,
  BluetoothEnableState: BluetoothEnableState,
  GpioAction: GpioAction,
  ProtectionZoneList: ProtectionZoneList,
  Gen3GpioPinId: Gen3GpioPinId,
  CartesianWaypoint: CartesianWaypoint,
  MapGroupList: MapGroupList,
  TrajectoryErrorReport: TrajectoryErrorReport,
  FullIPv4Configuration: FullIPv4Configuration,
  ProtectionZoneNotificationList: ProtectionZoneNotificationList,
  FullUserProfile: FullUserProfile,
  TrajectoryInfo: TrajectoryInfo,
  NetworkHandle: NetworkHandle,
  Sequence: Sequence,
  MapGroupHandle: MapGroupHandle,
  WrenchCommand: WrenchCommand,
  Base_RotationMatrixRow: Base_RotationMatrixRow,
  BridgeType: BridgeType,
  TransformationRow: TransformationRow,
  ControllerElementEventType: ControllerElementEventType,
  MappingList: MappingList,
  JointTrajectoryConstraintType: JointTrajectoryConstraintType,
  JointNavigationDirection: JointNavigationDirection,
  Finger: Finger,
  ControllerElementHandle_identifier: ControllerElementHandle_identifier,
  Base_ControlMode: Base_ControlMode,
  Map: Map,
  ControllerConfiguration: ControllerConfiguration,
  ProtectionZoneHandle: ProtectionZoneHandle,
  ChangeTwist: ChangeTwist,
  ControllerBehavior: ControllerBehavior,
  LimitationType: LimitationType,
  BaseFeedback: BaseFeedback,
  ActuatorCustomData: ActuatorCustomData,
  BaseCyclic_CustomData: BaseCyclic_CustomData,
  ActuatorFeedback: ActuatorFeedback,
  BaseCyclic_ServiceVersion: BaseCyclic_ServiceVersion,
  ActuatorCommand: ActuatorCommand,
  BaseCyclic_Command: BaseCyclic_Command,
  BaseCyclic_Feedback: BaseCyclic_Feedback,
  NotificationHandle: NotificationHandle,
  Empty: Empty,
  UserProfileHandle: UserProfileHandle,
  UARTDeviceIdentification: UARTDeviceIdentification,
  Connection: Connection,
  CartesianReferenceFrame: CartesianReferenceFrame,
  ArmState: ArmState,
  NotificationType: NotificationType,
  DeviceTypes: DeviceTypes,
  UARTStopBits: UARTStopBits,
  SafetyHandle: SafetyHandle,
  UARTParity: UARTParity,
  Unit: Unit,
  SafetyNotification: SafetyNotification,
  Timestamp: Timestamp,
  SafetyStatusValue: SafetyStatusValue,
  UARTConfiguration: UARTConfiguration,
  CountryCodeIdentifier: CountryCodeIdentifier,
  NotificationOptions: NotificationOptions,
  UARTSpeed: UARTSpeed,
  Permission: Permission,
  UARTWordLength: UARTWordLength,
  DeviceHandle: DeviceHandle,
  CountryCode: CountryCode,
  CartesianTransform: CartesianTransform,
  CartesianReferenceFrameInfo: CartesianReferenceFrameInfo,
  TwistLinearSoftLimit: TwistLinearSoftLimit,
  ControlConfigurationEvent: ControlConfigurationEvent,
  ControlConfig_ControlMode: ControlConfig_ControlMode,
  ControlConfig_ControlModeNotification: ControlConfig_ControlModeNotification,
  DesiredSpeeds: DesiredSpeeds,
  JointAccelerationSoftLimits: JointAccelerationSoftLimits,
  KinematicLimitsList: KinematicLimitsList,
  GravityVector: GravityVector,
  ToolConfiguration: ToolConfiguration,
  ControlConfig_ServiceVersion: ControlConfig_ServiceVersion,
  TwistAngularSoftLimit: TwistAngularSoftLimit,
  ControlConfig_ControlModeInformation: ControlConfig_ControlModeInformation,
  ControlConfigurationNotification: ControlConfigurationNotification,
  ControlConfig_JointSpeeds: ControlConfig_JointSpeeds,
  AngularTwist: AngularTwist,
  ControlConfig_Position: ControlConfig_Position,
  LinearTwist: LinearTwist,
  JointSpeedSoftLimits: JointSpeedSoftLimits,
  PayloadInformation: PayloadInformation,
  KinematicLimits: KinematicLimits,
  SafetyThreshold: SafetyThreshold,
  SerialNumber: SerialNumber,
  PowerOnSelfTestResult: PowerOnSelfTestResult,
  DeviceConfig_ServiceVersion: DeviceConfig_ServiceVersion,
  SafetyEnable: SafetyEnable,
  PartNumber: PartNumber,
  PartNumberRevision: PartNumberRevision,
  DeviceType: DeviceType,
  CalibrationStatus: CalibrationStatus,
  DeviceConfig_SafetyLimitType: DeviceConfig_SafetyLimitType,
  CalibrationResult: CalibrationResult,
  CalibrationItem: CalibrationItem,
  CalibrationParameter: CalibrationParameter,
  DeviceConfig_CapSenseMode: DeviceConfig_CapSenseMode,
  RunMode: RunMode,
  SafetyInformationList: SafetyInformationList,
  MACAddress: MACAddress,
  FirmwareVersion: FirmwareVersion,
  DeviceConfig_CapSenseConfig: DeviceConfig_CapSenseConfig,
  ModelNumber: ModelNumber,
  SafetyInformation: SafetyInformation,
  CalibrationParameter_value: CalibrationParameter_value,
  CalibrationElement: CalibrationElement,
  SafetyConfigurationList: SafetyConfigurationList,
  CapSenseRegister: CapSenseRegister,
  RunModes: RunModes,
  IPv4Settings: IPv4Settings,
  SafetyConfiguration: SafetyConfiguration,
  Calibration: Calibration,
  SafetyStatus: SafetyStatus,
  BootloaderVersion: BootloaderVersion,
  RebootRqst: RebootRqst,
  DeviceManager_ServiceVersion: DeviceManager_ServiceVersion,
  DeviceHandles: DeviceHandles,
  GripperConfig_SafetyIdentifier: GripperConfig_SafetyIdentifier,
  RobotiqGripperStatusFlags: RobotiqGripperStatusFlags,
  MotorCommand: MotorCommand,
  GripperCyclic_MessageId: GripperCyclic_MessageId,
  GripperCyclic_Feedback: GripperCyclic_Feedback,
  CustomDataUnit: CustomDataUnit,
  GripperCyclic_CustomData: GripperCyclic_CustomData,
  GripperCyclic_ServiceVersion: GripperCyclic_ServiceVersion,
  MotorFeedback: MotorFeedback,
  GripperCyclic_Command: GripperCyclic_Command,
  I2CWriteParameter: I2CWriteParameter,
  I2CRegisterAddressSize: I2CRegisterAddressSize,
  I2CDeviceIdentification: I2CDeviceIdentification,
  GPIOState: GPIOState,
  GPIOPull: GPIOPull,
  I2CMode: I2CMode,
  InterconnectConfig_SafetyIdentifier: InterconnectConfig_SafetyIdentifier,
  I2CWriteRegisterParameter: I2CWriteRegisterParameter,
  UARTPortId: UARTPortId,
  I2CConfiguration: I2CConfiguration,
  EthernetSpeed: EthernetSpeed,
  EthernetDuplex: EthernetDuplex,
  InterconnectConfig_ServiceVersion: InterconnectConfig_ServiceVersion,
  InterconnectConfig_GPIOConfiguration: InterconnectConfig_GPIOConfiguration,
  EthernetConfiguration: EthernetConfiguration,
  I2CData: I2CData,
  GPIOMode: GPIOMode,
  I2CDevice: I2CDevice,
  I2CReadParameter: I2CReadParameter,
  GPIOIdentifier: GPIOIdentifier,
  GPIOValue: GPIOValue,
  I2CReadRegisterParameter: I2CReadRegisterParameter,
  EthernetDeviceIdentification: EthernetDeviceIdentification,
  I2CDeviceAddressing: I2CDeviceAddressing,
  GPIOIdentification: GPIOIdentification,
  EthernetDevice: EthernetDevice,
  InterconnectCyclic_Feedback: InterconnectCyclic_Feedback,
  InterconnectCyclic_MessageId: InterconnectCyclic_MessageId,
  InterconnectCyclic_Feedback_tool_feedback: InterconnectCyclic_Feedback_tool_feedback,
  InterconnectCyclic_Command_tool_command: InterconnectCyclic_Command_tool_command,
  InterconnectCyclic_CustomData_tool_customData: InterconnectCyclic_CustomData_tool_customData,
  InterconnectCyclic_CustomData: InterconnectCyclic_CustomData,
  InterconnectCyclic_Command: InterconnectCyclic_Command,
  InterconnectCyclic_ServiceVersion: InterconnectCyclic_ServiceVersion,
  BaseType: BaseType,
  CompleteProductConfiguration: CompleteProductConfiguration,
  BrakeType: BrakeType,
  ProductConfigurationEndEffectorType: ProductConfigurationEndEffectorType,
  WristType: WristType,
  ArmLaterality: ArmLaterality,
  VisionModuleType: VisionModuleType,
  InterfaceModuleType: InterfaceModuleType,
  EndEffectorType: EndEffectorType,
  ModelId: ModelId,
  FocusPoint: FocusPoint,
  Resolution: Resolution,
  VisionEvent: VisionEvent,
  Option: Option,
  FrameRate: FrameRate,
  VisionConfig_RotationMatrix: VisionConfig_RotationMatrix,
  SensorFocusAction: SensorFocusAction,
  OptionValue: OptionValue,
  OptionIdentifier: OptionIdentifier,
  ManualFocus: ManualFocus,
  TranslationVector: TranslationVector,
  IntrinsicProfileIdentifier: IntrinsicProfileIdentifier,
  VisionConfig_RotationMatrixRow: VisionConfig_RotationMatrixRow,
  SensorIdentifier: SensorIdentifier,
  BitRate: BitRate,
  Sensor: Sensor,
  DistortionCoefficients: DistortionCoefficients,
  ExtrinsicParameters: ExtrinsicParameters,
  SensorFocusAction_action_parameters: SensorFocusAction_action_parameters,
  SensorSettings: SensorSettings,
  VisionNotification: VisionNotification,
  VisionConfig_ServiceVersion: VisionConfig_ServiceVersion,
  IntrinsicParameters: IntrinsicParameters,
  OptionInformation: OptionInformation,
  FocusAction: FocusAction,
  FollowCartesianTrajectoryActionGoal: FollowCartesianTrajectoryActionGoal,
  FollowCartesianTrajectoryGoal: FollowCartesianTrajectoryGoal,
  FollowCartesianTrajectoryResult: FollowCartesianTrajectoryResult,
  FollowCartesianTrajectoryAction: FollowCartesianTrajectoryAction,
  FollowCartesianTrajectoryActionFeedback: FollowCartesianTrajectoryActionFeedback,
  FollowCartesianTrajectoryActionResult: FollowCartesianTrajectoryActionResult,
  FollowCartesianTrajectoryFeedback: FollowCartesianTrajectoryFeedback,
};
