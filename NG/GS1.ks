//--SETUP VARIABLES--//
LOCAL _RECOVERYOPTION is "ASDS". // ASDS,RTLS,EXPEND are options
runOncePath("0:/NG/GS1Functions.ks").
LOCAL _GROUNDTARGETOPTION is _LANDINGTARGET.
LOCAL _SECONDSTAGEPROCESSOR is processor("StageTwoProcessor").
SET steeringManager:maxstoppingtime to 1.25.
SET steeringManager:rollts to 7.5.
//--MAIN--//
FROM {local countdown is 30.} UNTIL countdown = 0 STEP {SET countdown to countdown - 1.} DO 
{
    clearScreen.
    print "UPD 1 T-00:00:" + countdown.
    wait 1.
}
stage.
LOCK throttle to 1.
set ship:control:pilotmainthrottle to 1.
IF _RECOVERYOPTION = "ASDS" {
    UNTIL ship:mass - ship:drymass <= 67.81 {
        LOCAL _N IS 1.16071E-8 * alt:radar^2 - 0.0017375 * alt:radar + 91.9643.
        LOCK STEERING to heading(90,_N,90).
    }
} else if _RECOVERYOPTION = "RTLS" {
    UNTIL ship:mass - ship:drymass <= 128 {
        LOCAL _N IS 1.16071E-8 * alt:radar^2 - 0.0017375 * alt:radar + 91.9643.
        LOCK STEERING to heading(90,_N,90).
    }
}
LOCK throttle to 0.
_FUELCHECK().
wait 3.1.
IF _SECONDSTAGEPROCESSOR:connection:sendmessage("Seperation") {
    print("Seperation Command Sent to Second Stage").
}
set ship:control:pilotmainthrottle to 0.
wait 6.5.
RCS ON.

IF _RECOVERYOPTION = "ASDS" {
    toggle ag1.
    _PREDESCENTGLIDE().
    WAIT UNTIL ALT:RADAR <= 47000.
    _ENTRYBURN().
    WAIT UNTIL ALT:RADAR <= 5020.
    _LANDINGOPS().
    _FUELCHECK().
} ELSE IF _RECOVERYOPTION = "RTLS" {
    LOCK STEERING TO -vxcl(up:vector,ship:velocity:surface).
    set steeringManager:maxstoppingtime to 2.25.
    wait 15.
    TOGGLE AG1.
    set steeringManager:maxstoppingtime to 1.25.
    _BOOSTBACKBURN(_GROUNDTARGETOPTION).
    _INTERMITGUIDANCE().
    WAIT UNTIL ALT:RADAR <= 42000.
    _ENTRYBURN().
    WAIT UNTIL ALT:RADAR <= 3950.
    _LANDINGOPS().
    _FUELCHECK().
} ELSE {
    print("Booster OPS complete").
}