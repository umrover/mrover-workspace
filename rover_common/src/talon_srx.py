from enum import Enum


BITMASK = 0xFFFFFFF0


STATUS_1 = 0x02041400
STATUS_2 = 0x02041440
STATUS_3 = 0x02041480
STATUS_4 = 0x020414C0
STATUS_5 = 0x02041500
STATUS_6 = 0x02041540
STATUS_7 = 0x02041580
STATUS_8 = 0x020415C0
STATUS_9 = 0x02041600
STATUS_10 = 0x02041640
STATUS_11 = 0x02041680


CONTROL_1 = 0x02040000
CONTROL_2 = 0x02040040
CONTROL_3 = 0x02040080
CONTROL_5 = 0x02040100
CONTROL_6 = 0x02040140


PARAM_REQUEST = 0x02041800
PARAM_RESPONSE = 0x02041840
PARAM_SET = 0x02041880


class kLimitSwitchOverride(Enum):
    UseDefaultsFromFlash = 1
    DisableFwd_DisableRev = 4
    DisableFwd_EnableRev = 5
    EnableFwd_DisableRev = 6
    EnableFwd_EnableRev = 7


class TalonControlMode(Enum):
    kThrottle = 0
    kFollowerMode = 5
    kVoltageMode = 4
    kPositionMode = 1
    kSpeedMode = 2
    kCurrentMode = 3
    kMotionProfileMode = 6
    kMotionMagic = 7
    kDisabled = 15


class FeedbackDevice(Enum):
    QuadEncoder = 0
    AnalogPot = 2
    AnalogEncoder = 3
    EncRising = 4
    EncFalling = 5
    CtreMagEncoder_Relative = 6
    CtreMagEncoder_Absolute = 7
    PulseWidth = 8


class Param(Enum):
    ProfileParamSlot0_P = 1
    ProfileParamSlot0_I = 2
    ProfileParamSlot0_D = 3
    ProfileParamSlot0_F = 4
    ProfileParamSlot0_IZone = 5
    ProfileParamSlot0_CloseLoopRampRate = 6
    ProfileParamSlot1_P = 11
    ProfileParamSlot1_I = 12
    ProfileParamSlot1_D = 13
    ProfileParamSlot1_F = 14
    ProfileParamSlot1_IZone = 15
    ProfileParamSlot1_CloseLoopRampRate = 16
    ProfileParamSoftLimitForThreshold = 21
    ProfileParamSoftLimitRevThreshold = 22
    ProfileParamSoftLimitForEnable = 23
    ProfileParamSoftLimitRevEnable = 24
    OnBoot_BrakeMode = 31
    OnBoot_LimitSwitch_Forward_NormallyClosed = 32
    OnBoot_LimitSwitch_Reverse_NormallyClosed = 33
    OnBoot_LimitSwitch_Forward_Disable = 34
    OnBoot_LimitSwitch_Reverse_Disable = 35
    Fault_OverTemp = 41
    Fault_UnderVoltage = 42
    Fault_ForLim = 43
    Fault_RevLim = 44
    Fault_HardwareFailure = 45
    Fault_ForSoftLim = 46
    Fault_RevSoftLim = 47
    StckyFault_OverTemp = 48
    StckyFault_UnderVoltage = 49
    StckyFault_ForLim = 50
    StckyFault_RevLim = 51
    StckyFault_ForSoftLim = 52
    StckyFault_RevSoftLim = 53
    AppliedThrottle = 61
    CloseLoopErr = 62
    FeedbackDeviceSelect = 63
    RevMotDuringCloseLoopEn = 64
    ModeSelect = 65
    ProfileSlotSelect = 66
    RampThrottle = 67
    RevFeedbackSensor = 68
    LimitSwitchEn = 69
    LimitSwitchClosedFor = 70
    LimitSwitchClosedRev = 71
    SensorPosition = 73
    SensorVelocity = 74
    Current = 75
    BrakeIsEnabled = 76
    EncPosition = 77
    EncVel = 78
    EncIndexRiseEvents = 79
    QuadApin = 80
    QuadBpin = 81
    QuadIdxpin = 82
    AnalogInWithOv = 83
    AnalogInVel = 84
    Temp = 85
    BatteryV = 86
    ResetCount = 87
    ResetFlags = 88
    FirmVers = 89
    SettingsChanged = 90
    QuadFilterEn = 91
    PidIaccum = 93
    Status1FrameRate = 94
    Status2FrameRate = 95
    Status3FrameRate = 96
    Status4FrameRate = 97
    Status6FrameRate = 98
    Status7FrameRate = 99
    ClearPositionOnIdx = 100
    PeakPosOutput = 104
    NominalPosOutput = 105
    PeakNegOutput = 106
    NominalNegOutput = 107
    QuadIdxPolarity = 108
    Status8FrameRate = 109
    AllowPosOverflow = 110
    ProfileParamSlot0_AllowableClosedLoopErr = 111
    NumberPotTurns = 112
    NumberEncoderCPR = 113
    PwdPosition = 114
    AinPosition = 115
    ProfileParamVcompRate = 116
    ProfileParamSlot1_AllowableClosedLoopErr = 117
    Status9FrameRate = 118
    MotionProfileHasUnderrunErr = 119
    Reserved120 = 120
    LegacyControlMode = 121
    MotMag_Accel = 122
    MotMag_VelCruise = 123
    Status10FrameRate = 124
    CurrentLimThreshold = 125
    CustomParam0 = 137
    CustomParam1 = 138
    PersStorageSaving = 139
    ClearPositionOnLimitF = 144
    ClearPositionOnLimitR = 145
    NominalBatteryVoltage = 146
    SampleVelocityPeriod = 147
    SampleVelocityWindow = 148


def fxp_10_22_to_float(fxp, signed=False):
    CONVERSION_CONST = 0.0000002384185791015625
    raw = float(fxp)
    return raw * CONVERSION_CONST


def float_to_fxp_10_22(val, signed=False):
    CONVERSION_CONST = float(0x400000)
    return int(val * CONVERSION_CONST)


def sign_extend(x, b):
    if x >= (1 << (b - 1)):
        return x - (1 << b)
    else:
        return x
