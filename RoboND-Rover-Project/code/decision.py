import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    
    avg_angle = np.mean(Rover.nav_angles * 180/np.pi) 
    len_angle = len(Rover.nav_angles)
    avg_dist = np.mean(Rover.nav_dists) 
    
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Detect if we are stuck; should be when we are not moving and not picking up any rock samples 
        if abs(Rover.vel) < 0.02 and not Rover.picking_up and not Rover.near_sample :            
            Rover.halted += 1
        else:
            Rover.halted = 0
        
        if Rover.halted > 2:
                Rover.mode = 'stop' # we have been blocked for a while lets take action to get out

            
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                
                # Set steering to average angle clipped to the range +/- 15
                # Hug the wall to the right or to the left 
                # The target for steering is set based on the width the navigatable terrain 
                if Rover.vel > 0.2: # if we are moving at ok speed, lets try to stay close to the wall
                    if len_angle > 3000:  # this is the case when the terrain is wide 
                        goal = 30
                        if Rover.vel < Rover.max_vel:
                            Rover.throttle = 1
                    elif len_angle < 1000: # this is a narrow terrain
                        goal = 10
                    else: # this is the case if its average width terrain 
                        goal = 20
                    # adjust the goal depending on if wall is on right or left side
                    if avg_angle < 0:
                        goal = goal*-1
                    # if we are already hugging the wall dont steer
                    if avg_angle == goal:
                        Rover.steer = 0
                    else:
                        Rover.steer = -1*np.clip(goal - avg_angle, -15,15)
                else: # if we are slow then maybe we are too close to wall, lets just go to the median
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                #angle_adjust = np.random.random_sample()*5
                #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi)+angle_adjust, -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if abs(Rover.vel) > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif abs(Rover.vel) <= 0.2:
                Rover.throttle = 0
                # Release the brake to allow turning
                Rover.brake = 0

                # Now we're stopped and we have vision data to see if there's a path forward
                if (len(Rover.nav_angles) < Rover.go_forward) or Rover.halted:
                    if Rover.halted > 5 and Rover.halted < 50: # go reverse out of a bad blocked state, if its too bad (>50) lets just do a full turn
                        Rover.throttle = -0.1* np.random.randint(1,10)

                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    else: 
                        Rover.steer = -15
                    
                    #Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain and too many obstacles in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward and not Rover.halted:
                    # Set throttle back to stored value
                    if Rover.throttle == 0:
                        Rover.throttle = Rover.throttle_set
                        # Release the brake
                        Rover.brake = 0
                        # Set steer to mean angle
                        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                        Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

