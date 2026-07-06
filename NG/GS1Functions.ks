GLOBAL ASDS is vessel("LSC"):geoposition.
//GLOBAL ASDS is latlng(26.8855473125067,-61.8203611357222).
GLOBAL LZ1 is latlng(28.4905428584226,-81.2039369051358).
GLOBAL LZ2 is latlng(28.4815950011763,-81.2188321882249).
GLOBAL RecovOpt is "ASDS".
GLOBAL _LANDINGTARGET is "LZ-2".
function _GETIMPACT {
    if addons:tr:hasimpact {
        return addons:tr:impactpos.
    } ELSE {
        return ship:geoPosition.
    }
}

GLOBAL FUNCTION _STEERTOTARGET { // Again, credit to Saturn Aerospace
    PARAMETER _TARGET.
    PARAMETER _MAXAOA.
    PARAMETER _ERRSCALING is 1.
    LOCAL _IMPACTPOS is 0.
    if addons:tr:hasimpact {
        set _IMPACTPOS to _GETIMPACT():position.
    } else {
        set _IMPACTPOS to ship:position.
    }
	LOCAL _POSERROR is _IMPACTPOS - _TARGET:position.
	local _VELOCITYVECTOR is -ship:velocity:surface.
    local _RESULT is _VELOCITYVECTOR + _POSERROR * _ERRSCALING.

    if vAng(_RESULT, _VELOCITYVECTOR) > _MAXAOA {
        set _RESULT to _VELOCITYVECTOR:normalized + tan(_MAXAOA) * _POSERROR:normalized.
    }
    return lookDirUp(_RESULT, facing:topvector).
}
function CalcMaxTWR {
    LOCAL TWR is SHIP:MAXTHRUST / (9.80688 * SHIP:MASS).
    return TWR.
}
function HoverPID {
    parameter altit.
    local LBGP is pidLoop(
        1.5,  // Kp
        0.75, // Ki
        2,   // Kd
        0.1, // Min
        1   // Max
    ).
    set LBGP:setpoint to altit.
    lock throttle to LBGP:update(time:seconds,alt:radar).
}
function TWRtoThrottle {
    parameter twr.
    LOCAL maxTwr is CalcMaxTWR().
    LOCAL throttleTarget is twr/maxTwr.
    if twr > maxTwr {
        return 1.
    }
    return throttleTarget.
}
GLOBAL FUNCTION _INTERMITGUIDANCE {
	//set ASDS to vessel("LSC"):geoposition.
	IF RecovOpt = "RTLS" {
		IF _LANDINGTARGET = "LZ-1" {LOCK STEERING to _STEERTOTARGET(LZ1,28).}
		IF _LANDINGTARGET = "LZ-2" {LOCK STEERING to _STEERTOTARGET(LZ2,28).}
	} ELSE IF RecovOpt = "ASDS" {
		LOCK STEERING to _STEERTOTARGET(ASDS,35,1.16).
	} ELSE {
        LOCK STEERING to lookDirUp(ship:srfretrograde:vector,ship:facing:upvector).
	}
    set steeringManager:maxstoppingtime to 2.
}
GLOBAL FUNCTION _ENTRYBURN {
	set steeringManager:torqueepsilonmin to 0.0000002. // 1000x more precise over default
	//set ASDS to vessel("LSC"):geoposition.
    // Insert Next(or previous burn mode) here.
	rcs off.
	 // shuts down the 4 non-landing engines
	IF RecovOpt = "RTLS" {
		IF _LANDINGTARGET = "LZ-1" {LOCK STEERING to _STEERTOTARGET(LZ1,-13.3,1.2).}
		IF _LANDINGTARGET = "LZ-2" {LOCK STEERING to _STEERTOTARGET(LZ2,-13.3,1.2).}
	} ELSE IF RecovOpt = "ASDS" {
		LOCK STEERING to _STEERTOTARGET(ASDS,-23).
	} ELSE {
        LOCK STEERING to lookDirUp(ship:srfretrograde:vector,ship:facing:upvector).
	}
    LOCK throttle to 1.
    WAIT UNTIL abs(ship:airspeed) <= 1225.
    set ASDS to vessel("LSC"):geoposition.
    LOCK throttle to 0.
    _INTERMITGUIDANCE().
	set steeringManager:maxstoppingtime to 0.8.
}
GLOBAL FUNCTION _LANDINGOPS { // Combination of HerrCraziDev and Saturn Aerospace code (to make it actually work)
    set steeringManager:maxstoppingtime to 0.75.
    parameter defaultRadarOffset is 7, gearDeployTime is 5.7.
    local radarOffset is 7.
    clearscreen.

    if defaultRadarOffset <> 7
    {
	    set radarOffset to defaultRadarOffset.
    }

    if abs(ship:verticalspeed) < 1
    {
	        set radarOffset to alt:radar.	 							// The value of alt:radar when landed (on gear)
    } else if radarOffset = 0 {
	    set radarOffset to defaultRadarOffset.
	    print "Warning : pilot engaged while in flight, radar offset will be set to 7 (XASR-3).".
    }

    //Distance from impact point, else vessel's altitude
    if addons:tr:available and addons:tr:hasimpact {
	    lock impactDist to addons:tr:impactpos:distance.
    } else {
	    lock impactDist to alt:radar - radarOffset.
	    print "Warning : impact position not available. You should (re)install Trajectories, or maybe takeoff.".
    }
	local S1AO is 20.
    lock g to constant:g * body:mass / body:radius^2.						// Gravity (m/s^2)
	lock trueradar to alt:radar - S1AO - 30.
    lock shipVel to ship:velocity:surface:mag.								// Vessel's total velocity
    lock maxDecel to (ship:availablethrust / ship:mass) - g.				// Maximum deceleration possible (m/s^2)
    lock stopDist to ship:verticalspeed^2 / (2 * maxDecel).	// The distance the burn will require
    lock idealThrottle to stopDist / trueradar.							// Throttle required for perfect hoverslam
    lock impactTime to trueradar / abs(shipVel).							// Time until impact, used for landing gear

    print "Radar offset : " at (0, terminal:width).
    print radarOffset at (16, terminal:width).


when ship:verticalspeed < -1 then
{
	print "Preparing for autolanding...".

	rcs on.
	sas off.
	brakes on.
	IF RecovOpt = "RTLS" {
		IF _LANDINGTARGET = "LZ-1" {LOCK STEERING to _STEERTOTARGET(LZ1,10).}
		IF _LANDINGTARGET = "LZ-2" {LOCK STEERING to _STEERTOTARGET(LZ2,10).}
	} ELSE IF RecovOpt = "ASDS" {
		LOCK STEERING to _STEERTOTARGET(ASDS,21.5,1.6).
	} ELSE {
        LOCK STEERING to lookDirUp(ship:srfretrograde:vector,ship:facing:upvector).
	}
	when impactTime < gearDeployTime then
	{
	}

	when trueradar < stopDist - S1AO + 320 then
	{
		set ASDS to vessel("LSC"):geoposition.
		print "Performing autolanding".
		print idealThrottle.
		print shipVel.
		print stopDist.
		IF RecovOpt = "RTLS" {
			IF _LANDINGTARGET = "LZ-1" {LOCK STEERING to _STEERTOTARGET(LZ1,-1).}
			IF _LANDINGTARGET = "LZ-2" {LOCK STEERING to _STEERTOTARGET(LZ2,-1).}
	    } ELSE IF RecovOpt = "ASDS" {
		    //LOCK STEERING to _STEERTOTARGET(ASDS,-12).
	    } ELSE {
            LOCK STEERING to lookDirUp(ship:srfretrograde:vector,ship:facing:upvector).
	    }
		set addons:tr:descentangles to LIST(0,0,0,0).
		lock STEERING to _STEERTOTARGET(ASDS,-6.7,1.4).
		lock throttle to 0.9.
		when abs(ship:verticalspeed) <= 24 then {
			lock STEERING to _STEERTOTARGET(ASDS,-6.7).
		    lock throttle to (idealThrottle + TWRtoThrottle(0.97)).
		} 
	    when alt:radar <= 140 then {
			LOCK STEERING to up.
			gear on. 
			//toggle ag2.
			wait 1.2.
			lock throttle to TWRtoThrottle(1).
			//LOCK STEERING to lookDirUp(ship:srfretrograde:vector,ship:facing:upvector).
			wait 3.5.
			lock throttle to TWRtoThrottle(0.67).
			when ship:verticalspeed <= -3.4 then {
				lock throttle to TWRtoThrottle(1).
			}
		}
		
		when impactTime < 2 then
		{
			lock impactDist to  alt:radar - radarOffset.
			print "Precision approach phase. Impact in 2s.".
		}

		when ship:status = "LANDED" OR ship:status = "SPLASHED" then
		{
			print "Autolanding completed".
			set ship:control:pilotmainthrottle to 0.
			lock throttle to 0.
			toggle ag2.
			toggle ag3.
			unlock steering.
			rcs off.
            print(ship:geoposition).
		}
	}
}


UNTIL ship:status = "LANDED" OR ship:status = "SPLASHED"
{
	print "SRF VEL  :   " + round(shipVel, 4)			 + " m/s              " at (0, terminal:height - 11).
	print "HOR VEL  :   " + round(ship:groundspeed, 4)	 + " m/s              " at (0, terminal:height - 10).
	print "VERT VEL :   " + round(ship:verticalspeed, 4) + " m/s              " at (0, terminal:height - 9).
	print "DESC RATE:   " + round( abs(ship:verticalspeed/ship:groundspeed), 2) + "              " at (0, terminal:height - 8).

	print "IMPACT           :   T+"	+ round(impactTime, 3)	+ " s              " 	at (0, terminal:height - 6).
	print "IMPACT DIST      :   "	+ round(trueradar, 2)	+ " m              " 	at (0, terminal:height - 5).
	print "MAX DECEL        :   "	+ round(maxDecel, 5)	+ " m/s²              " at (0, terminal:height - 4).
	print "S. BURN DIST     :   "	+ round(stopDist, 2)	+ " m              " 	at (0, terminal:height - 3).

	print "THROTTLE :   " + round(idealThrottle*100,2) + " %              " at (0, terminal:height - 1).
	print "SHIP STATUS :  " + ship:status + "." at (0, terminal:height - 12).
	WAIT 0.01.
}
} // Close Landing Burn
GLOBAL FUNCTION _PREDESCENTGLIDE {
	LOCK STEERING to heading(90,60,90).
	WAIT UNTIL eta:apoapsis < 35.
	addons:tr:settarget(vessel("LSC"):geoposition).
	BRAKES ON.
	LOCK STEERING to heading(90,90,90).
	WAIT UNTIL eta:apoapsis < 1.
	toggle ag4.
	set addons:tr:descentangles to LIST(0,10,7,-5).
	_INTERMITGUIDANCE().
}
GLOBAL FUNCTION _BOOSTBACKBURN {
	parameter RecoveryTarget.
	IF SHIP = kuniverse:activevessel {
		print "This vessel is the active vessel: Performing optimal boostback".
		IF RecoveryTarget = "LZ-1" {
			Until vxcl(up:vector,ship:velocity:surface):mag <= 15 {
                LOCK throttle to 0.8.
                LOCK STEERING TO -vxcl(up:vector,ship:velocity:surface).
                clearScreen.
                print (addons:tr:impactpos:position - LZ1:position):mag.
                //LOCK STEERING TO heading(92.2635,-172.5,90).
			}
			Until (addons:tr:impactpos:position - LZ1:position):mag <= 1300 {
                LOCK throttle to 0.8.
                LOCK STEERING TO vxcl(up:forevector, LZ1:position).
                clearScreen.
                print (addons:tr:impactpos:position - LZ1:position):mag.
                //LOCK STEERING TO heading(92.2635,-172.5,90).
			}
			LOCK throttle to 0.
			set ship:control:pilotmainthrottle to 0.
			BRAKES ON.
		}
		IF RecoveryTarget = "LZ-2" {
			Until vxcl(up:vector,ship:velocity:surface):mag <= 15 {
                LOCK throttle to 0.8.
                LOCK STEERING TO -vxcl(up:vector,ship:velocity:surface).
                clearScreen.
                print (addons:tr:impactpos:position - LZ2:position):mag.
                //LOCK STEERING TO heading(92.2635,-172.5,90).
			}
			Until (addons:tr:impactpos:position - LZ2:position):mag <= 1300 {
                LOCK throttle to 0.8.
                LOCK STEERING TO vxcl(up:forevector, LZ2:position).
                clearScreen.
                print (addons:tr:impactpos:position - LZ2:position):mag.
                //LOCK STEERING TO heading(92.2635,-172.5,90).
			}
			LOCK throttle to 0.
			set ship:control:pilotmainthrottle to 0.
			BRAKES ON.
		}
	} ELSE {

	}
}
GLOBAL FUNCTION _FUELCHECK {
	clearScreen.
	print "DRY MASS: " +ship:drymass at(0,terminal:height-1).
	print "WET MASS:" + ship:wetmass at(0,terminal:height-2).
	LOCAL _FUELMASS is ship:mass - ship:drymass.
	print "FUEL MASS:" + _FUELMASS at(0,terminal:height-4).
}