import glob
import os
import sys
import random

import numpy as np
import pygame

IM_WIDTH = 1280
IM_HEIGHT = 1024

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla

# ==============================================================================
# -- Helper Function -----------------------------------------------------------
# ==============================================================================

def draw_image(surface, image, semanticseg):

    if semanticseg == True:
        image.convert(carla.ColorConverter.CityScapesPalette)

    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    surface.blit(image_surface, (0, 0))

# ==============================================================================
# -- Game Loop -----------------------------------------------------------------
# ==============================================================================

actor_list = []
import time
def game_loop():
    # INIT
    pygame.init()
    pygame.font.init()
    try:
        display = pygame.display.set_mode((IM_WIDTH, IM_HEIGHT),pygame.HWSURFACE | pygame.DOUBLEBUF)

        client = carla.Client('localhost', 2000)            #Connect to server
        client.set_timeout(2.0)

        world = client.get_world()
        world.unload_map_layer(carla.MapLayer.All)          #Remove map layers https://carla.readthedocs.io/en/latest/core_map/#carla-maps
        map = world.get_map()
        spectator = world.get_spectator()
        tm = client.get_trafficmanager(8000)

        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        actor_list = world.get_actors()
        ego = None
        print(actor_list)
        for speed_sign in actor_list.filter('vehicle.tesla*'):
            ego = speed_sign 
            print("printai:" + str(ego))
        id = 0 
        ego_vehicle = ego
        for actor in world.get_actors():
            if actor.attributes.get('type') == 'ego_vehicle':
                ego_vehicle = actor
                id = actor.get_attribute()
                break
        blueprint_library = world.get_blueprint_library()   #Get access to actors https://carla.readthedocs.io/en/0.9.12/bp_library/

        # VEHICLE
        vehicle_bp = blueprint_library.find('vehicle.tesla.cybertruck')
        spawn_points = map.get_spawn_points()
        destination = random.choice(spawn_points).location
        transform = carla.Transform(destination)
        # ego_vehicle = world.spawn_actor(vehicle_bp, transform)
        # actor_list.append(ego_vehicle)

        # ego_vehicle.set_autopilot(True,8000)

        # SEMANTIC SEGMENTATION BIRDS-EYE
        # ss_camera_bp = blueprint_library.find('sensor.camera.semantic_segmentation')
        # ss_camera_bp.set_attribute('image_size_x', f'{IM_WIDTH}')
        # ss_camera_bp.set_attribute('image_size_y', f'{IM_HEIGHT}')
        # ss_camera_bp.set_attribute('fov', '110')
        # # ss_cam_location = carla.Location(0,0,50)
        # # ss_cam_rotation = carla.Rotation(-90,0,0)
        # ss_cam_location = carla.Location(2,0,1)
        # ss_cam_rotation = carla.Rotation(0,0,0)
        # ss_cam_transform = carla.Transform(ss_cam_location, ss_cam_rotation)
        # ss_cam = world.spawn_actor(ss_camera_bp, ss_cam_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # # actor_list.append(ss_cam)
        # ss_cam.listen(lambda data: draw_image(display, data, True))
        
        # SPECTATOR CAMERA
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', f'{IM_WIDTH}')
        camera_bp.set_attribute('image_size_y', f'{IM_HEIGHT}')
        camera_bp.set_attribute('fov', '110')
        # cam_location = carla.Location(-10,0,5)
        # cam_location = carla.Location(-0.15,-0.4,1.2)
        cam_location = carla.Location(2,0,1)
        cam_rotation = carla.Rotation(0,180,0)
        cam_transform = carla.Transform(cam_location, cam_rotation)
        spectator_cam = world.spawn_actor(camera_bp, cam_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.SpringArm)
        # actor_list.append(spectator_cam)
        spectator_cam.listen(lambda data: draw_image(display, data, False))

        #LIDAR CAMERA
        lidar_cam = None
        # lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        # lidar_bp.set_attribute('channels',str(32))
        # lidar_bp.set_attribute('points_per_second',str(90000))
        # lidar_bp.set_attribute('rotation_frequency',str(40))
        # lidar_bp.set_attribute('range',str(20))
        # lidar_location = carla.Location(2,0,1)
        # lidar_rotation = carla.Rotation(0,0,0)
        # lidar_transform = carla.Transform(lidar_location,lidar_rotation)
        # lidar_sen = world.spawn_actor(lidar_bp,lidar_transform,attach_to=ego_vehicle,attachment_type=carla.AttachmentType.SpringArm)
        # lidar_sen.listen(lambda point_cloud: point_cloud.save_to_disk('tutorial/new_lidar_output3/%.6d.ply' % point_cloud.frame))
        # lidar_sen.listen(lambda data: draw_image(display, data, False))
        # UE4 SPECTATOR SETTING
        #spec_location = carla.Location(0,0,50)
        #spec_rotation = carla.Rotation(-90,0,0)
        running = True
        file_num = 0
        done_capturing = False

        while not done_capturing:
            file_num = file_num + 1
            image = spectator_cam.get_image()
            display.blit(image, (0,0))
            pygame.display.update()

            # Save every frame
            filename = "Snaps/%04d.png" % file_num
            pygame.image.save(image, filename)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done_capturing = True

        # Combine frames to make video
        os.system("avconv -r 8 -f image2 -i Snaps/%04d.png -y -qscale 0 -s 640x480 -aspect 4:3 result.avi")
        while running:
            pygame.display.flip()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    sys.exit()

        # while True:
            # VEHICLE CONTROL
            #vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-1.0))

            
            # UE4 SPECTATOR CAMERA in BEV
            #actor_transform = vehicle.get_transform()
            #actor_location = actor_transform.location
            #actor_rotation = actor_transform.rotation
            #spectator.set_transform(carla.Transform(actor_location + carla.Location(z=50), carla.Rotation(-90, 0, 0)))
            #spectator.set_transform(carla.Transform(actor_location + carla.Location(z=50), carla.Rotation(-90, actor_rotation.yaw, actor_rotation.roll))) #Not smooth


    finally:
        print('Destroying actors')
        # for actor in actor_list:
        #     actor.destroy()
        print('Actors destroyed')
        pygame.quit()
        print('Quit pygame')

# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================

def main():
    time.sleep(3)
    try:
        game_loop()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__ == '__main__':
    main()
