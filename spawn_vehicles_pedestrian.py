import glob
import os
import sys
import argparse
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from numpy import random
import logging

client = carla.Client('localhost', 2000)
client.set_timeout(2.0)
def draw_waypoints(world,waypoints, road_id=None, life_time=50.0):

  for waypoint in waypoints:

    if(waypoint.road_id == road_id):
      world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                   color=carla.Color(r=0, g=255, b=0), life_time=life_time,
                                   persistent_lines=True)

def spawn_vehicle(world,road_id,vehicle_name,waypoints):
    vehicle_blueprint = world.get_blueprint_library().filter(vehicle_name)[0]
    filtered_waypoints = []
    for waypoint in waypoints:
        if(waypoint.road_id == road_id):
            filtered_waypoints.append(waypoint)
    #print(filtered_waypoints[0].transform.location.x , filtered_waypoints[0].transform.location.y)
    spawn_point = filtered_waypoints[0].transform
    spawn_point.location.z += 2
    vehicle = world.spawn_actor(vehicle_blueprint, spawn_point)
    vehicle.set_autopilot()
    return vehicle

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Sensor tutorial')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--async',
        dest='sync',
        action='store_false',
        help='Asynchronous mode execution')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='pedestrians filter (default: "walker.pedestrian.*")')
    
    argparser.set_defaults(sync=True)

    args = argparser.parse_args()

    #args.width, args.height = [int(x) for x in args.res.split('x')]


    try:
        vehicles_list=[]
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)
        world=client.get_world()

        blueprintsWalkers = world.get_blueprint_library().filter(args.filterw)
        SpawnActor = carla.command.SpawnActor
        walkers_list = []
        all_id = []



        waypoints = world.get_map().generate_waypoints(distance=1.0)
        for i in range(0,10):
            draw_waypoints(world,waypoints, road_id=i, life_time=20)
        for i in range(3):
            vehicle=spawn_vehicle(world,10+i,'model3',waypoints)
            vehicles_list.append(vehicle)

        percentagePedestriansRunning = 0.0      # how many pedestrians will run
        percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
        # 1. take all the random locations to spawn
        spawn_points = []
        filtered_waypoints = []
        #for waypoint in waypoints:
        #    if(waypoint.road_id >= 10 and waypoint.road_id <=30):
        #        filtered_waypoints.append(waypoint)
        for i in range(30):
            spawn_point=carla.Transform()
            #spawn_point = filtered_waypoints[random.randint(len(filtered_waypoints))].transform
            loc = world.get_random_location_from_navigation()
            threshod=20
            while loc.x >= -threshod and loc.x<=threshod and loc.y>=-threshod and loc.y<=threshod:
                loc = world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        # 4. we put altogether the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if not args.sync :
            world.wait_for_tick()
        else:
            world.tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(all_id), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))
        print('spawned %d walkers, press Ctrl+C to exit.' % len(walkers_list))
        while True:
            if args.sync :
                world.tick()
            else:
                world.wait_for_tick()
    finally:
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])
if __name__ == '__main__':

    main()