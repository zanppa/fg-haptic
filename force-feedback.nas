# Variables, need to make controllable
var update_interval = 0.05;

# Defaults constants, will be overwritten in init
var aileron_max_deflection = 20.0*0.01745329;
var elevator_max_deflection = 20.0*0.01745329;
var rudder_max_deflection = 20.0*0.01745329;
var aileron_gain = 0.1;
var elevator_gain = 0.1;
var g_force_gain = 0.003;
var rudder_gain = 0.1;
var slip_gain = 1.0;
var stall_AoA = 18.0*0.01745329;
var pusher_start_AoA = 900.0*0.01745329;
var pusher_working_angle = 900.0*0.01745329;
var wing_shadow_AoA = 900.0*0.01745329;
var wing_shadow_angle = 900.0*0.01745329;
var stick_shaker_AoA = 16.0*0.01745329;

var enable_force_trim_aileron = nil;
var enable_force_trim_elevator = nil;
var enable_force_trim_rudder = nil;
var aileron_trim_prop = nil;
var elevator_trim_prop = nil;
var rudder_trim_prop = nil;
var trim_rate = 0.045;

var origAileronTrim = controls.aileronTrim;
var origElevatorTrim = controls.elevatorTrim;
var origRudderTrim = controls.rudderTrim;

var test_duration = 0.0;

###
# Update different forces

# Pilot G forces
var update_pilot_g = func(path) {
  var pilot_path = path.getNode("pilot");
  if(pilot_path == nil) return;  # Bail out if pilot forces are not supported

  # If gain is set to 0, effect is disabled so don't update
  #if(pilot_path.getNode("gain").getValue() < 0.001) return;

  var pilot_x = getprop("/accelerations/pilot/x-accel-fps_sec");  # Forwards, positive towards back
  var pilot_y = getprop("/accelerations/pilot/y-accel-fps_sec");  # Sideways, positive to right
  var pilot_z = getprop("/accelerations/pilot/z-accel-fps_sec");  # Downwards, positive down

  # 1 G = 1.0 at output

  if(pilot_x == nil) pilot_x = 0;
  else pilot_x = pilot_x / 32.174;

  if(pilot_y == nil) pilot_y = 0;
  else pilot_y = pilot_y / 32.174;

  if(pilot_z == nil) pilot_z = 0;
  else pilot_z = (pilot_z + 32.174) / 32.174;

  # TODO: Axis mapping!
  # Default mapping, in haptic +Y is backwards and +X is to the left
  var axis_x = pilot_path.getNode("x");
  if(axis_x != nil) axis_x.setValue(-pilot_y); # Needs to be inverted

  var axis_y = pilot_path.getNode("y");
  if(axis_y != nil) axis_y.setValue(pilot_x);

  var axis_z = pilot_path.getNode("z");
  if(axis_z != nil) axis_z.setValue(pilot_z);
};


# Forces acting on the control stick
var update_stick_forces = func(path) {
  var stick_force_path = path.getNode("stick-force");
  if(stick_force_path == nil) return;

  #if(stick_force_path.getNode("gain").getValue() < 0.001) return;

  var mode = 0;	# Default to normal mode
  var mode_path = stick_force_path.getNode("mode");
  if(mode_path != nil) mode = mode_path.getValue();

  var airspeed = getprop("/velocities/airspeed-kt");
  var AoA = getprop("/orientation/alpha-deg")*0.01745329;
  var slip_angle = getprop("/orientation/side-slip-deg")*0.01745329;
  var density = getprop("/environment/density-slugft3");
  var g_force = getprop("/accelerations/pilot/z-accel-fps_sec") + 32.174;

  # Basic forces from air flow, taking slip into account if desirable
  # 0.5 * air_density * airspeed^2 * drag_coeff (assuming 2) * Area
  var base_force = density * airspeed * airspeed;

  # In normal mode the spring effect force gain depends only on
  # airspeed, density and surface area. Stick calculates the actual
  # force based on stick deviation from center
  var aileron_force = base_force * aileron_gain;
  var elevator_force = base_force * elevator_gain;
  var rudder_force = base_force * rudder_gain;

  if(mode == 1) {
    # In alternate mode the control surface forces are done using constant force
    # so we need to calculate the actual force based on surface angles etc.

    # This also takes trim into account as it works as an offset to the force zero angle

    var aileron_angle = (getprop("/controls/flight/aileron") + aileron_trim_prop.getValue()) * aileron_max_deflection;
    var elevator_angle = (getprop("/controls/flight/elevator") + elevator_trim_prop.getValue()) * elevator_max_deflection;
    var rudder_angle = (getprop("/controls/flight/rudder") + rudder_trim_prop.getValue()) * rudder_max_deflection;

    # TODO: Check whether AoA should be + or -!
    elevator_angle = elevator_angle + AoA;
    rudder_angle = rudder_angle - slip_angle;

    slip_gain = 1.0 - slip_gain * math.sin(slip_angle);

    aileron_force = aileron_force * math.sin(aileron_angle) * slip_gain;
  
    elevator_force = elevator_force * math.sin(elevator_angle) * slip_gain;
    elevator_force = elevator_force + g_force_gain * g_force;

    rudder_force = rudder_force * math.sin(rudder_angle);
  }  


  # Stall condition, assuming rudder wont stall
  # A smooth "step" function going from 0 to 1, i.e. at deep stall
  # the stick forces are zero
  var stall_coeff = (AoA / stall_AoA);
  stall_coeff = stall_coeff * stall_coeff * stall_coeff * stall_coeff;
  if(stall_coeff > 1.0) stall_coeff = 1.0;

  elevator_force = elevator_force * (1 - stall_coeff);
  aileron_force = aileron_force * (1 - stall_coeff);


  # Wing shadowing effect, parabolic function going from 1 to 0 and back to 1
  if(wing_shadow_AoA != nil and wing_shadow_angle != nil)
  {
    if(AoA > wing_shadow_AoA and AoA < wing_shadow_AoA + wing_shadow_angle)
    {
      var shadow = (AoA - wing_shadow_AoA - 0.5 * wing_shadow_angle);
      shadow = shadow / (wing_shadow_angle * 0.5);
      shadow = shadow * shadow;
      if(shadow > 1.0) shadow = 1.0;
      elevator_force = elevator_force * shadow;
    }
  }

  # Trim is handled differently if normal mode is enabled
  # trim changes the stick zero offset position
  if(mode == 0) {
    var axis_x = stick_force_path.getNode("trim-aileron", 1);
    if(axis_x != nil) axis_x.setValue(aileron_trim_prop.getValue());
    var axis_y = stick_force_path.getNode("trim-elevator", 1);
    if(axis_y != nil) axis_y.setValue(elevator_trim_prop.getValue());
    var axis_z = stick_force_path.getNode("trim-rudder", 1);
    if(axis_z != nil) axis_z.setValue(rudder_trim_prop.getValue());
  }
  
  # Stick pusher
  if(pusher_start_AoA != nil and pusher_working_angle != nil) {
    if(AoA > pusher_start_AoA)
      elevator_force = elevator_force - ((AoA - pusher_start_AoA) / pusher_working_angle);
  }

  # TODO: Axis mapping!
  var axis_x = stick_force_path.getNode("aileron", 1);
  if(axis_x != nil) axis_x.setValue(-aileron_force);

  var axis_y = stick_force_path.getNode("elevator", 1);
  if(axis_y != nil) axis_y.setValue(-elevator_force);

  var axis_z = stick_force_path.getNode("rudder", 1);
  if(axis_z != nil) axis_z.setValue(rudder_force);


  # Stick shaker
  if(AoA > stick_shaker_AoA)
  {
    setprop("/haptic/stick-shaker/trigger", 1);
  } else {
    setprop("/haptic/stick-shaker/trigger", 0);
  }
};


# Ground rumble
var update_ground_rumble = func(path) {
  var rumble_path = path.getNode("ground-rumble");
  if(rumble_path == nil) return;  # Bail out if rumble forces are not supported

  var period = 0.0;

  # Only do stuff if there is weigh on wheels
  if(getprop("/gear/gear/wow")) {
    var groundspeed = getprop("/velocities/groundspeed-kt");
    if(groundspeed > 3.0 )
      period = 15000.0 / groundspeed;
  }

  var period_node = rumble_path.getNode("period");
  if(period_node != nil) period_node.setValue(period); # Needs to be inverted
};



# Test mode
var run_test_mode = func(path) {
  var x = 0.0;
  var y = 0.0;
  var z = 0.0;
  var shaker = 0;

  var stick_force_path = path.getNode("stick-force");

  if(test_duration < 10)
  {
    # Test constant force
    x = math.sin(3.14159*test_duration/5.0);
    y = math.cos(3.14159*test_duration/5.0);
    z = math.sin(3.14159*test_duration/5.0);
  }
  else if(test_duration < 16)
  {
    # Test stick shaker
    shaker = 1;
  }
  else
  {
    # Loop
    test_duration = 0.0;
  }

  # Set parameters
  if(stick_force_path != nil) {
    stick_force_path.getNode("aileron").setValue(x);
    stick_force_path.getNode("elevator").setValue(y);
    stick_force_path.getNode("rudder").setValue(z);
  }
  setprop("/haptic/stick-shaker/trigger", shaker);

  test_duration = test_duration + update_interval;
};



# Main loop
var update_forces = func {
  # Loop through every device
  var haptic_node = props.globals.getNode("/haptic");
  var test_mode = getprop("/haptic/test-mode");

  if(!test_mode)
  {
    if(haptic_node != nil)
    {
      update_pilot_g(haptic_node);
      update_stick_forces(haptic_node);
      update_ground_rumble(haptic_node);
    }
  } else {
    run_test_mode(haptic_node);
  }

  # Reset timer
  settimer(update_forces, update_interval);
};


###
# Trim functions
controls.aileronTrim = func(rate) {
  if(!enable_force_trim_aileron.getValue())
    origAileronTrim(rate);
  else
    controls.slewProp("/haptic/force-trim-aileron", trim_rate*rate);
};
controls.elevatorTrim = func(rate) {
  if(!enable_force_trim_elevator.getValue())
    origElevatorTrim(rate);
  else
    controls.slewProp("/haptic/force-trim-elevator", trim_rate*rate);
};
controls.rudderTrim = func(rate) {
  if(!enable_force_trim_rudder.getValue())
    origRudderTrim(rate);
  else
    controls.slewProp("/haptic/force-trim-rudder", trim_rate*rate);
};




###
# Read aircraft properties when fdm is ready
_setlistener("/sim/signals/fdm-initialized", func {
  # Read aircraft setup
  aileron_max_deflection = getprop("/haptic/aircraft-setup/aileron-max-deflection-deg")*0.01745329;
  elevator_max_deflection = getprop("/haptic/aircraft-setup/elevator-max-deflection-deg")*0.01745329;
  rudder_max_deflection = getprop("/haptic/aircraft-setup/rudder-max-deflection-deg")*0.01745329;

  aileron_gain = getprop("/haptic/aircraft-setup/aileron-gain");
  elevator_gain = getprop("/haptic/aircraft-setup/elevator-gain");
  rudder_gain = getprop("/haptic/aircraft-setup/rudder-gain");
  g_force_gain = getprop("/haptic/aircraft-setup/g-force-gain");
  slip_gain = getprop("/haptic/aircraft-setup/slip-gain");

  stall_AoA = getprop("/haptic/aircraft-setup/stall-AoA")*0.01745329;
  pusher_start_AoA = getprop("/haptic/aircraft-setup/pusher-start-AoA")*0.01745329;
  pusher_working_angle = getprop("/haptic/aircraft-setup/pusher-working-angle-deg")*0.01745329;
  wing_shadow_AoA = getprop("/haptic/aircraft-setup/wing-shadow-AoA")*0.01745329;
  wing_shadow_angle = getprop("/haptic/aircraft-setup/wing-shadow-angle-deg")*0.01745329;
  stick_shaker_AoA = getprop("/haptic/aircraft-setup/stick-shaker-AoA")*0.01745329;
});


###
# Main initialization
_setlistener("/sim/signals/nasal-dir-initialized", func {

  # Add default parameters to property tree
  # TODO: Update constants from aircraft setup?
  props.globals.initNode("/haptic/aircraft-setup/aileron-max-deflection-deg", aileron_max_deflection/0.01745329, "DOUBLE");
  props.globals.initNode("/haptic/aircraft-setup/elevator-max-deflection-deg", elevator_max_deflection/0.01745329, "DOUBLE");
  props.globals.initNode("/haptic/aircraft-setup/rudder-max-deflection-deg", rudder_max_deflection/0.01745329, "DOUBLE");

  props.globals.initNode("/haptic/aircraft-setup/aileron-gain", aileron_gain, "DOUBLE");
  props.globals.initNode("/haptic/aircraft-setup/elevator-gain", elevator_gain, "DOUBLE");
  props.globals.initNode("/haptic/aircraft-setup/rudder-gain", rudder_gain, "DOUBLE");
  props.globals.initNode("/haptic/aircraft-setup/g-force-gain", g_force_gain, "DOUBLE");
  props.globals.initNode("/haptic/aircraft-setup/slip-gain", slip_gain, "DOUBLE");

  props.globals.initNode("/haptic/aircraft-setup/stall-AoA", stall_AoA/0.01745329, "DOUBLE");
  props.globals.initNode("/haptic/aircraft-setup/pusher-start-AoA", pusher_start_AoA/0.01745329, "DOUBLE");
  props.globals.initNode("/haptic/aircraft-setup/pusher-working-angle-deg", pusher_working_angle/0.01745329, "DOUBLE");
  props.globals.initNode("/haptic/aircraft-setup/wing-shadow-AoA", wing_shadow_AoA/0.01745329, "DOUBLE");
  props.globals.initNode("/haptic/aircraft-setup/wing-shadow-angle-deg", wing_shadow_angle/0.01745329, "DOUBLE");
  props.globals.initNode("/haptic/aircraft-setup/stick-shaker-AoA", stick_shaker_AoA/0.01745329, "DOUBLE");

  enable_force_trim_aileron = props.globals.initNode("/haptic/enable-force-trim-aileron", 0, "BOOL");
  enable_force_trim_elevator = props.globals.initNode("/haptic/enable-force-trim-elevator", 0, "BOOL");
  enable_force_trim_rudder = props.globals.initNode("/haptic/enable-force-trim-rudder", 0, "BOOL");
  aileron_trim_prop = props.globals.initNode("/haptic/force-trim-aileron", 0.0, "DOUBLE");
  elevator_trim_prop = props.globals.initNode("/haptic/force-trim-elevator", 0.0, "DOUBLE");
  rudder_trim_prop = props.globals.initNode("/haptic/force-trim-rudder", 0.0, "DOUBLE");
  props.globals.initNode("/haptic/stick-force/trim-aileron", 0.0, "DOUBLE");
  props.globals.initNode("/haptic/stick-force/trim-elevator", 0.0, "DOUBLE");
  props.globals.initNode("/haptic/stick-force/trim-rudder", 0.0, "DOUBLE");

  props.globals.initNode("/haptic/stick-force/gain", 0.0, "DOUBLE");
  props.globals.initNode("/haptic/stick-force/mode", 0, "INT");
  
  props.globals.initNode("/haptic/ground-rumble/gain", 0.0, "DOUBLE");
  props.globals.initNode("/haptic/ground-rumble/mode", 0, "INT");
  
  props.globals.initNode("/haptic/test-mode", 0, "BOOL");

 
  # Add dialog to menu
  props.globals.getNode("/sim/menubar/default/menu[9]/item[99]/enabled", 1).setBoolValue(1);
  props.globals.getNode("/sim/menubar/default/menu[9]/item[99]/name", 1).setValue("force-feedback");
  props.globals.getNode("/sim/menubar/default/menu[9]/item[99]/label", 1).setValue("Force feedback options");
  props.globals.getNode("/sim/menubar/default/menu[9]/item[99]/binding/command", 1).setValue("dialog-show");
  props.globals.getNode("/sim/menubar/default/menu[9]/item[99]/binding/dialog-name", 1).setValue("force-feedback");
  #gui.menubind("Force feedback", "dialogs.force-feedback.open()");

  # Reload menu so the new item will appear
  fgcommand("reinit", props.Node.new({subsystem:"gui"}));

  # Set timer for main loop
  settimer(update_forces, update_interval);
});
