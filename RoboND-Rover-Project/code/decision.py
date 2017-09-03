import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function

def get_steer_angle(Rover):
    avg_angle = np.mean(Rover.nav_angles * 180/np.pi) 
    len_angle = len(Rover.nav_angles)
    avg_dist = np.mean(Rover.nav_dists) 
    
    if len_angle > 3000:  # this is the case when the terrain is wide 
         goal = -30
    elif len_angle < 1000: # this is a narrow terrain
        goal = -10
    else: # this is the case if its average width terrain 
         goal = -20               
    # if we are already hugging the wall dont steer
    if avg_angle == goal:
        steer = 0
    else:
        steer = -1*np.clip(goal - avg_angle, -15,15)
        return steer
    
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    
    avg_angle = np.mean(Rover.nav_angles * 180/np.pi) 
    len_angle = len(Rover.nav_angles)
    avg_dist = np.mean(Rover.nav_dists) 
    yaw_diff = Rover.prev_yaw - Rover.yaw
    dist_travelled = np.sqrt((Rover.prev_pos[0] - Rover.pos[0])**2 + \
                                        (Rover.prev_pos[1] - Rover.pos[1])**2)

    
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
    
            # TODO, if we are not mapping for a while, do something 
        if Rover.perc_mapped  == Rover.prev_perc_mapped: 
            Rover.nomap_count +=1
        else:
            Rover.nomap_count=0
        
        # Try to detect if we are going around in circles (clockwsie or anticlockwise
        if not((Rover.prev_yaw <= 360 and Rover.yaw >=0) or (Rover.prev_yaw>=0 and Rover.yaw <=360)): #special case, skip it
            if yaw_diff >0:
                Rover.clockwisecount +=1
                Rover.anticlockwisecount = 0
            else:
                Rover.clockwisecount = 0 
                Rover.anticlockwisecount += 1

        # Are we going around in circles 
        if Rover.anticlockwisecount > 300 or Rover.clockwisecount > 300 : 
            Rover.mode = 'circles'
        elif Rover.mode == 'circles': # We were in circles lasttime, just reset to FWD state
            Rover.mode = 'forward'
        
        # Detect if we are stuck; should be when we are not moving and not picking up any rock samples
        elif (abs(Rover.vel) < 0.02 or dist_travelled < 0.01) and not Rover.picking_up and not Rover.near_sample :    
            if Rover.mode == 'forward': # Rover is supposed to move FWD but is stuck
                Rover.halted += 1
                if Rover.halted > 3:
                    Rover.mode = 'stuck' # we have been stuck for a while lets take action to get out
            elif Rover.mode == 'stuck':
                Rover.mode = 'forward'
                Rover.throttle = Rover.throttle_set*3
        else:
            Rover.halted = 0
            if Rover.mode == 'stuck':
                Rover.mode = 'forward'
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                   
        if Rover.total_time < 2.0: # special case for start of rover sim, just move forward
            Rover.mode = 'forward'
        
        
        # Check for Rover.mode status
        if Rover.mode == 'circles': # We are stuck in a circle, do something else
            Rover.steer = 0 # some other angle than 15
            Rover.clockwisecount = 0
            Rover.anticlockwisecount = 0
              
        elif Rover.mode == 'forward': 
 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    if Rover.vel < 0.2:
                        Rover.throttle = Rover.throttle_set*3
                    else:
                        Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                
                Rover.brake = 0
                Rover.steer = get_steer_angle(Rover)
 
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
                if (len(Rover.nav_angles) < Rover.go_forward): 
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain and too many obstacles in front then go!
                elif len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    if Rover.throttle == 0:
                        Rover.throttle = Rover.throttle_set
                        # Release the brake
                        Rover.brake = 0
                        # Set steer to mean angle
                        Rover.steer = get_steer_angle(Rover)
                        Rover.mode = 'forward'
                        
        # The Rover is stuck, let get out of this jam 
        elif Rover.mode == 'stuck':
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = -15
            Rover.halted = 0 
            #Rover.mode = 'stop'
            #if (len(Rover.nav_angles) < Rover.go_forward):
            #    Rover.steer = -15
            #else:
            #    Rover.mode = 'forward'
            #    Rover.brake = 0
               
            #if Rover.halted > 5 and Rover.halted < 50: # go reverse out of a bad blocked state, else lets just do a full turn
            #    Rover.throttle = -0.1* np.random.randint(1,10)
             
            #else:
            #    Rover.throttle = 0
            #    Rover.brake=0
            #    if Rover.vel == 0
            #        Rover.steer = -15

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

