print("AWAITING SEPERATION PING").
WAIT UNTIL NOT CORE:MESSAGES:EMPTY.
SET _RECEIVED TO CORE:MESSAGES:POP.
IF _RECEIVED:CONTENT = "Seperation" {
    lock throttle to 0.
    stage.
    PRINT("Seperation command received").
}
wait 5.25.
clearScreen.
print("SES").
RCS OFF.
lock throttle to 1.
set ship:control:pilotmainthrottle to 1.
LOCK STEERING to heading(90,11,90).
wait 15.
stage.
UNTIL ALT:APOAPSIS >= 140000 {
    clearScreen.
    print "Ascent".
    SAS OFF.
    LOCK throttle to 0.9.
    LOCK STEERING to heading(90,11,90).
    wait 0.5.
}
LOCK STEERING to heading(90,-9.5,90).
WAIT UNTIL ALT:periapsis >= 115000.
lock throttle to 0.
set ship:control:pilotmainthrottle to 0.
