#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Script that render multiple sensors in the same pygame window
By default, it renders four cameras, one LiDAR and one Semantic LiDAR.
It can easily be configure for any different number of sensors. 
To do that, check lines 290-308.
"""

import glob
import os
import sys
from math import sqrt
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import random
import time
import numpy as np
global record_on
record_on=0
global no
no=1
global saved_rgb
saved_rgb=[]
try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
    from pygame.locals import K_SPACE
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_g
    from pygame.locals import K_h
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')
def control(car):
    """
    Applies control to main car based on pygame pressed keys.
    Will return True If ESCAPE is hit, otherwise False to end main loop.
    """
    keys = pygame.key.get_pressed()
    if keys[K_ESCAPE]:
        return True

    control = car.get_control()
    control.throttle = 0
    if keys[K_w]:
        control.throttle = 1
        control.reverse = False
    elif keys[K_s]:
        control.throttle = 1
        control.reverse = True
    if keys[K_a]:
        control.steer = max(-1., min(control.steer - 0.05, 0))
    elif keys[K_d]:
        control.steer = min(1., max(control.steer + 0.05, 0))
    else:
        control.steer = 0
    control.hand_brake = keys[K_SPACE]
    car.apply_control(control)
    return False
class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()

class DisplayManager:
    def __init__(self, grid_size, window_size):
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)

        self.grid_size = grid_size
        self.window_size = window_size
        self.sensor_list = []

    def get_window_size(self):
        return [int(self.window_size[0]), int(self.window_size[1])]

    def get_display_size(self):
        return [int(self.window_size[0]/self.grid_size[1]), int(self.window_size[1]/self.grid_size[0])]

    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    def get_sensor_list(self):
        return self.sensor_list

    def render(self):
        if not self.render_enabled():
            return

        for s in self.sensor_list:
            s.render()

        pygame.display.flip()

    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

    def render_enabled(self):
        return self.display != None

class SensorManager:
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos):
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.timer = CustomTimer()
        self.time_processing = 0.0
        self.tics_processing = 0
        self.display_man.add_sensor(self)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'RGBCameraleft':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image_left)

            return camera
        elif sensor_type == 'RGBCameramid':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image_mid)

            return camera
        elif sensor_type == 'RGBCameraright':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image_right)

            return camera
        elif sensor_type == 'LiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('range', '100')
            lidar_bp.set_attribute('dropoff_general_rate', lidar_bp.get_attribute('dropoff_general_rate').recommended_values[0])
            lidar_bp.set_attribute('dropoff_intensity_limit', lidar_bp.get_attribute('dropoff_intensity_limit').recommended_values[0])
            lidar_bp.set_attribute('dropoff_zero_intensity', lidar_bp.get_attribute('dropoff_zero_intensity').recommended_values[0])

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_lidar_image)

            return lidar
        
        elif sensor_type == 'SemanticLiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
            lidar_bp.set_attribute('range', '100')

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_semanticlidar_image)

            return lidar
        
        elif sensor_type == "Radar":
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            for key in sensor_options:
                radar_bp.set_attribute(key, sensor_options[key])

            radar = self.world.spawn_actor(radar_bp, transform, attach_to=attached)
            radar.listen(self.save_radar_image)

            return radar
        
        else:
            return None

    def get_sensor(self):
        return self.sensor

    def save_rgb_image_left(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        #if self.display_man.render_enabled():
        #    self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1
        if self.tics_processing%8==0 and record_on==1:
            image.save_to_disk('_out/left_camera/%03d/%08d'%(no , image.frame))
    
    def save_rgb_image_mid(self, image):
        t_start = self.timer.time()
        global saved_rgb
        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1
        if self.tics_processing%8==0 and record_on==1:
            saved_rgb.append(image)
            #image.save_to_disk('_out/mid_camera/%03d/%08d' %(no , image.frame))

    def save_rgb_image_right(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        #if self.display_man.render_enabled():
        #    self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1
        if self.tics_processing%8==0 and record_on==1:
            image.save_to_disk('_out/right_camera/%03d/%08d' %(no , image.frame))

    def save_lidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0*float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        #if self.display_man.render_enabled():
        #    self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1
        if self.tics_processing%8==0 and record_on==1:
            image.save_to_disk('_out/lidar/%03d/%08d' %(no , image.frame))

    def save_semanticlidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0*float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 6), 6))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_radar_image(self, radar_data):
        t_start = self.timer.time()
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (len(radar_data), 4))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    def destroy(self):
        self.sensor.destroy()

def normalized(vec):
    norm=sqrt(vec.x**2+vec.y**2+vec.z**2)
    vec.x=vec.x/norm
    vec.y=vec.y/norm
    vec.z=vec.z/norm
    return vec

def find_road(road_points,pos,acceleration):
    if acceleration.x==0 and acceleration.y==0 and acceleration.z==0:
        return "vehicle is stop"
    for w in road_points:
        w_pos=w.transform.location
        if abs(w_pos.x-pos.x)<=1 and abs(w_pos.y-pos.y)<=1 and abs(w_pos.z-pos.z)<=1:
            next_nodes=w.next(1)
            if len(next_nodes)>=3:
                next_node=next_nodes[2].transform.location
            else:
                next_node=next_nodes[0].transform.location
            next_node.x-=w_pos.x
            next_node.y-=w_pos.y
            next_node.z-=w_pos.z
            acceleration=normalized(acceleration)
            cc=normalized(next_node)
            # if same direction
            if abs(acceleration.x-cc.x)<=0.2 and abs(acceleration.y-cc.y)<=0.2 and abs(acceleration.z-cc.z)<=0.2:
                return w
            if w.is_junction:
                return "On junction"
    return "c"

def cal_junct(road_points,waypoint,pos):
    ll=waypoint.next_until_lane_end(2)
    last=ll[-1].transform.location
    return last
    #last_2=ll[len(ll)-2].transform.location
    #last.x+=(last.x-last_2.x)*20
    #last.y+=(last.y-last_2.y)*20
    #last.z+=(last.z-last_2.z)*20
    #for ww in road_points:
    #    ww_pos=ww.transform.location
    #    if abs(ww_pos.x-last.x)<=3 and abs(ww_pos.y-last.y)<=3 and abs(ww_pos.z-last.z)<=3 and ww.is_junction:
    #        return ww_pos

def run_simulation(args, client):
    """This function performed one test run using the args parameters
    and connecting to the carla client passed.
    """

    display_manager = None
    vehicle = None
    vehicle_list = []
    timer = CustomTimer()

    try:

        # Getting the world and
        world = client.get_world()
        original_settings = world.get_settings()

        if args.sync:
            traffic_manager = client.get_trafficmanager(8000)
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)


        # Instanciating the vehicle to which we attached the sensors
        bp = world.get_blueprint_library().filter('charger2020')[0]
        vehicle = world.spawn_actor(bp, random.choice(world.get_map().get_spawn_points()))
        vehicle_list.append(vehicle)
        #vehicle.set_autopilot(True)

        # Display Manager organize all the sensors an its display in a window
        # If can easily configure the grid and the total window size
        display_manager = DisplayManager(grid_size=[1,1], window_size=[args.width, args.height])

        # Then, SensorManager can be used to spawn RGBCamera, LiDARs and SemanticLiDARs as needed
        # and assign each of them to a grid position, 
        # SensorManager(world, display_manager, 'RGBCameraleft', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=-60)), 
        #               vehicle, {}, display_pos=[0, 1])
        SensorManager(world, display_manager, 'RGBCameramid', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)), 
                      vehicle, {}, display_pos=[0, 0])
        # SensorManager(world, display_manager, 'RGBCameraright', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+60)), 
        #               vehicle, {}, display_pos=[0, 2])
        #SensorManager(world, display_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=180)), 
        #              vehicle, {}, display_pos=[1, 1])

        # SensorManager(world, display_manager, 'LiDAR', carla.Transform(carla.Location(x=0, z=2.4)), 
        #               vehicle, {'channels' : '64', 'range' : '50',  'points_per_second': '100000', 'rotation_frequency': '20'}, display_pos=[1, 0])
        #SensorManager(world, display_manager, 'SemanticLiDAR', carla.Transform(carla.Location(x=0, z=2.4)), 
        #              vehicle, {'channels' : '64', 'range' : '100', 'points_per_second': '100000', 'rotation_frequency': '20'}, display_pos=[1, 2])
        # state 0: find which road , 
        # 1 : already find calculate junction distance 
        # 2 : on junction, keeps find if find road not junction to state 0
        state=0
        #Simulation loop
        global record_on
        global no
        global saved_rgb
        call_exit = False
        time_init_sim = timer.time()
        find_on=0
        if find_on==1:
            road_points = world.get_map().generate_waypoints(distance=4.0)
        while True:
            # Carla Tick
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()
            if find_on==1:
                position=vehicle.get_location()
                acc=vehicle.get_acceleration()
            #start=((acc.x==0) and (acc.y==0)and (acc.z==0))
            if find_on==1:
                if state ==0:
                    this_w=find_road(road_points,position,acc)
                    try:
                        if this_w.is_junction:
                            state=0
                        else:
                            state=1
                            distance_last=1000
                    except:
                        pass
                elif state ==1:
                    try:
                        ww_pos=cal_junct(road_points,this_w,position)
                        distance=sqrt((ww_pos.x-position.x)**2+(ww_pos.y-position.y)**2+(ww_pos.z-position.z)**2)
                        if distance>distance_last:
                            state=0
                        distance_last=distance
                        #print("------------------------\n", end='\r')
                        #print("Which lane: \n",this_w.lane_id, end='\r')
                        print(distance, end='\r')
                        #print("------------------------", end='\r')
                        if distance<=5:
                            state=0
                    except:
                        state=0

            
            # Render received data
            display_manager.render()
            #print("current time : ",timer.time())
            if find_on==1:
                if this_w=="c":
                    print("can not find road", end='\r')
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key==K_g:
                        record_on+=1
                        if record_on%2==0:
                            record_on=0
                            print("---------------")
                            print("End recording!")
                            print("Starting output!")
                            print("---------------")
                            for image in saved_rgb:
                                image.save_to_disk('_out/mid_camera/%03d/%08d' %(no , image.frame))
                            saved_rgb.clear()
                            no+=1
                        else:
                            print("---------------")
                            print("Start recording!")
                            print("---------------")
            if control(vehicle):
                return
            if call_exit:
                break

    finally:
        if display_manager:
            display_manager.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])

        world.apply_settings(original_settings)



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
    argparser.set_defaults(sync=True)
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='640x320',
        help='window resolution (default: 640x320)')

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)

        run_simulation(args, client)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
