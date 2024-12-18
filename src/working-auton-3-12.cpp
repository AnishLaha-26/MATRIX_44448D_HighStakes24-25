//     //put this after op control
    
//     chassis.setPose(0, 0, 0); //this is intial set position (robot start zone)
//     chassis.moveToPoint(0,-17,1000,{.forwards = false}, false); //initial move forward to clear rings
//     chassis.moveToPoint(0,-10,1000,{.forwards = true}, false); //
//     chassis.moveToPose(
//         18,                 // x-coordinate
//         -11,                 // y-coordinate
//         -93,                // theta (heading in degrees)
//         1600,               // timeout in milliseconds
//         {.forwards = false, .horizontalDrift = 8, .lead = 0.6,}      
//     );
    
//     chassis.moveToPoint(18.5, -8 ,2000, {.forwards = false}, false);

//     intake1.move_relative(-375, 600); // Move intake1 360 degrees
//     intake2.move_relative(-375, 600); // Move intake2 360 degrees
//      // Wait for the motors to complete their movement
//     while (fabs(intake1.get_position() - intake1.get_target_position()) > 5 || 
//            fabs(intake2.get_position() - intake2.get_target_position()) > 5) {
//         pros::delay(10); // Check every 10ms
//    }

//   chassis.moveToPoint(13, 0,1000, {.forwards = false}, false);
//   chassis.turnToHeading(-134,500);
//   intake1.move_velocity(-600);
//   intake2.move_velocity(-600);
//   chassis.moveToPoint(-15, -40,2500, {.forwards = true}, false);
//   intake1.move_velocity(0);
//   intake2.move_velocity(0);
//   mogoToggle();
//   chassis.moveToPoint(-25,-33,2000);