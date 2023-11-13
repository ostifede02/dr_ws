import pinocchio as pin

def is_Collision(robot, q, collision_pair):
    ## checking for collisions
    # Compute for each pair of collision
    pin.updateGeometryPlacements(robot.model, robot.data, robot.collision_model, robot.collision_data, q)
    pin.computeCollision(robot.collision_model, robot.collision_data, collision_pair[0]) # chain 2
    pin.computeCollision(robot.collision_model, robot.collision_data, collision_pair[1]) # chain 1

    if(robot.collision_data.collisionResults[0].isCollision()):
        print(f"Collision detected: {robot.collision_model.collisionPairs[0]}")
        return True
    elif(robot.collision_data.collisionResults[1].isCollision()):
        print(f"Collision detected: {robot.collision_model.collisionPairs[1]}")
        return True
    
    return False