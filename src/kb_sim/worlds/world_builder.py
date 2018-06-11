import random
from sys import argv

tabstop = '  '
def createRoad(name, pos, x, y, z, indent):
    link =  '\n' + tabstop * indent + '<link name="%s">' % name
    link += '\n' + tabstop * (indent+1) + '<pose>%d %d %d %d %d %d</pose>' % pos
    link += '\n' + tabstop * (indent+1) + '<visual name="visual">'
    link += '\n' + tabstop * (indent+2) + '<geometry>'
    link += '\n' + tabstop * (indent+3) + '<box>'
    link += '\n' + tabstop * (indent+4) + '<size>%d %d %f</size>' % (x, y, z)
    link += '\n' + tabstop * (indent+3) + '</box>'
    link += '\n' + tabstop * (indent+2) + '</geometry>'
    link += '\n' + tabstop * (indent+2) + '<material>'
    link += '\n' + tabstop * (indent+3) + '<script>'
    link += '\n' + tabstop * (indent+4) + '<uri>model://urban/materials/scripts/</uri>'
    link += '\n' + tabstop * (indent+4) + '<uri>model://urban/materials/textures/</uri>'
    link += '\n' + tabstop * (indent+4) + '<name>asphalt/image</name>'
    link += '\n' + tabstop * (indent+3) + '</script>'
    link += '\n' + tabstop * (indent+2) + '</material>'
    link += '\n' + tabstop * (indent+1) + '</visual>'
    link += '\n' + tabstop * indent + '</link>'
    return link

def createRoads(size, num_horizontal_roads, num_vertical_roads, road_width, road_height, indent):
    z_offset = .4
    horizonal_distance = size[0] + 40
    vertical_distance = size[1] + 40
    centers = [[],[]]
    centers[0].append(-size[1]/2 + road_width/2)
    roads = createRoad('h0', (0,centers[0][-1],z_offset,0,0,0), horizonal_distance, road_width, road_height, indent)
    h_edges = [-size[1]/2 + road_width]
    for i in range(1,num_horizontal_roads-1):
        centers[0].append(i * size[1]/(num_horizontal_roads-1) - size[1]/2)
        roads += createRoad(
            'h%d' % i, (0,centers[0][-1],z_offset,0,0,0),
            horizonal_distance, road_width, road_height, indent)
        h_edges.append(i * size[1]/(num_horizontal_roads-1) - size[1]/2 + road_width/2)
    centers[0].append(size[1]/2 - road_width/2)
    roads += createRoad(
        'h%d' % (num_horizontal_roads-1), (0,centers[0][-1],z_offset,0,0,0),
        horizonal_distance, road_width, road_height, indent)
    h_edges.append(size[1]/2 - road_width)
    centers[1].append(-size[0]/2 + road_width/2)
    roads += createRoad('v0', (centers[1][-1],0,z_offset,0,0,0), road_width, vertical_distance, road_height, indent)
    v_edges = [-size[0]/2 + road_width]
    for i in range(1,num_vertical_roads-1):
        centers[1].append(i * size[0]/(num_vertical_roads-1) - size[0]/2)
        roads += createRoad(
            'v%d' % i, (centers[1][-1],0,z_offset,0,0,0),
            road_width, vertical_distance, road_height, indent)
        v_edges.append(i * size[0]/(num_vertical_roads-1) - size[0]/2 + road_width/2)
    centers[1].append(size[0]/2 - road_width/2)
    roads += createRoad(
        'v%d'%(num_vertical_roads-1), (centers[1][-1],0,z_offset,0,0,0),
        road_width, vertical_distance, road_height, indent)
    v_edges.append(size[0]/2 - road_width)
    num_spaces = (num_horizontal_roads-1)*(num_vertical_roads-1)
    open_areas = []
    for i in range(len(h_edges)-1):
        for j in range(len(v_edges)-1):
            open_areas.append([
                h_edges[i],v_edges[j],
                size[0]/(num_vertical_roads-1) - road_width * num_vertical_roads,
                size[1]/(num_horizontal_roads-1) - road_width * num_horizontal_roads])
    vehicle_nodes = []
    vehicle_edges = []
    for i in centers[0]:
        for j in centers[1]:
            vehicle_nodes.append((j,i))
            node_num = len(vehicle_nodes)
            #add horizontal intersections
            if node_num > 1:
                if vehicle_nodes[-2][1] == vehicle_nodes[-1][1]:
                    vehicle_edges.append((node_num - 2, node_num - 1))
            print(vehicle_nodes[-1])

    # add in vertical intersections
    vertical_edges = int(len(vehicle_nodes)/num_horizontal_roads)
    if num_horizontal_roads % 2 != 0:
        vertical_edges += num_vertical_roads
    for i in range(vertical_edges):
        vehicle_edges.append((i, i+num_vertical_roads))
    vehicle_waypoints = [vehicle_nodes, vehicle_edges]
    return roads, open_areas, vehicle_waypoints

def createBuilding(name, pos, length, width, height, indent):
    link =  '\n' + tabstop * indent + '<link name="%s">' % name
    link += '\n' + tabstop * (indent+1) + '<pose>%d %d %d %d %d %d</pose>' % pos
    link += '\n' + tabstop * (indent+1) + '<visual name="visual">'
    link += '\n' + tabstop * (indent+2) + '<geometry>'
    link += '\n' + tabstop * (indent+3) + '<box>'
    link += '\n' + tabstop * (indent+4) + '<size>%d %d %f</size>' % (length, width, height)
    link += '\n' + tabstop * (indent+3) + '</box>'
    link += '\n' + tabstop * (indent+2) + '</geometry>'
    link += '\n' + tabstop * (indent+2) + '<material>'
    link += '\n' + tabstop * (indent+3) + '<script>'
    link += '\n' + tabstop * (indent+4) + '<uri>model://urban/materials/scripts/</uri>'
    link += '\n' + tabstop * (indent+4) + '<uri>model://urban/materials/textures/</uri>'
    link += '\n' + tabstop * (indent+4) + '<name>building/image</name>'
    link += '\n' + tabstop * (indent+3) + '</script>'
    link += '\n' + tabstop * (indent+2) + '</material>'
    link += '\n' + tabstop * (indent+1) + '</visual>'
    link += '\n' + tabstop * indent + '</link>'
    return link

def createBuildings(space, num_buildings, indent):
    buildings = ''
    padding = 10
    weighted_sum = 0
    for i in range(num_buildings+1):
        weighted_sum += (num_buildings+1)/2 - abs(i - (num_buildings+1)/2)
    rand = random.randint(1,weighted_sum)
    for i in range(num_buildings+1):
        rand -= (num_buildings+1)/2 - abs(i - (num_buildings+1)/2)
        if rand <= 0:
            buildings_in_top_row = i
            break
    # print(buildings_in_top_row)
    start_x = space[1] + padding
    end_x = space[1] + space[2] - padding
    start_y = space[0] + padding
    end_y = space[0] + space[3] - padding
    building_dim = []
    buildings_built = 0
    for i in range(buildings_in_top_row):
        try:
            width = random.randint(10,int((end_y - start_y) * 2/3))
            length = random.randint(10,end_x - start_x)
            buildings_built += 1
        except:
            continue
        height = random.randint(20,60)
        buildings += createBuilding(
            'b%d'%i, (start_x + length/2, start_y + width/2, height/2, 0, 0, 0),
            length, width, height, indent)
        building_dim.append([start_x, start_x+length, start_y, start_y + width])
        start_x += length + padding
        padding = random.randint(10,20)

    start_x = space[1] + padding
    end_x = space[1] + space[2] - padding
    start_y = space[0] + padding
    end_y = space[0] + space[3] - padding
    for i in range(num_buildings - buildings_built):
        try:
            length = random.randint(10,end_x - start_x)
        except:
            continue
        start_y = space[0] + padding
        for j in range(len(building_dim)):
            if start_x + length < building_dim[j][0]:
                break
            if start_x > building_dim[j][1]:
                continue
            start_y = max(start_y, building_dim[j][3] + padding)
        width = random.randint(10,int(end_y - start_y))
        height = random.randint(20,60)
        buildings += createBuilding(
            'b%d'%i, (start_x + length/2, end_y - width/2, height/2, 0, 0, 0),
            length, width, height, indent)
        building_dim.append([start_x, start_x+length, end_y - width, end_y])
        start_x += length + padding
        padding = random.randint(10,20)
    return buildings

def createAllBuildings(open_spaces, building_per_space, indent):
    buildings = ''
    for i in range(len(open_spaces)):
        buildings += createBuildings(open_spaces[i], 5, indent)
    return buildings

def createGround(
        pos, size, num_vertical_roads, num_horizontal_roads, num_buildings,
        road_width, road_height, indent):
    ground  = '\n' + tabstop * indent + '<model name="ground_plane">'
    ground += '\n' + tabstop * (indent+1) + '<pose>%d %d %d %d %d %f</pose>' % pos
    ground += '\n' + tabstop * (indent+1) + '<static>1</static>'
    roads, open_spaces, vehicle_waypoints = createRoads(size, num_horizontal_roads, num_vertical_roads, road_width, road_height, indent+1)
    ground += roads
    ground += createAllBuildings(open_spaces, num_buildings, (indent+1))
    ground += '\n' + tabstop * (indent+1) + '<link name="link">'
    ground += '\n' + tabstop * (indent+2) + '<collision name="collision">'
    ground += '\n' + tabstop * (indent+3) + '<geometry>'
    ground += '\n' + tabstop * (indent+4) + '<plane>'
    ground += '\n' + tabstop * (indent+5) + '<normal>0 0 1</normal>'
    ground += '\n' + tabstop * (indent+5) + '<size>%d %d</size>' % (size[0], size[1])
    ground += '\n' + tabstop * (indent+4) + '</plane>'
    ground += '\n' + tabstop * (indent+3) + '</geometry>'
    ground += '\n' + tabstop * (indent+3) + '<surface>'
    ground += '\n' + tabstop * (indent+4) + '<friction>'
    ground += '\n' + tabstop * (indent+5) + '<ode>'
    ground += '\n' + tabstop * (indent+6) + '<mu>100</mu>'
    ground += '\n' + tabstop * (indent+6) + '<mu2>50</mu2>'
    ground += '\n' + tabstop * (indent+5) + '</ode>'
    ground += '\n' + tabstop * (indent+4) + '</friction>'
    ground += '\n' + tabstop * (indent+4) + '<bounce/>'
    ground += '\n' + tabstop * (indent+4) + '<contact>'
    ground += '\n' + tabstop * (indent+5) + '<ode/>'
    ground += '\n' + tabstop * (indent+4) + '</contact>'
    ground += '\n' + tabstop * (indent+3) + '</surface>'
    ground += '\n' + tabstop * (indent+3) + '<max_contacts>10</max_contacts>'
    ground += '\n' + tabstop * (indent+2) + '</collision>'
    ground += '\n' + tabstop * (indent+2) + '<visual name="visual">'
    ground += '\n' + tabstop * (indent+3) + '<cast_shadows>0</cast_shadows>'
    ground += '\n' + tabstop * (indent+3) + '<geometry>'
    ground += '\n' + tabstop * (indent+4) + '<plane>'
    ground += '\n' + tabstop * (indent+5) + '<normal>0 0 1</normal>'
    ground += '\n' + tabstop * (indent+5) + '<size>%f %f</size>' % (size[0] + 40, size[1] + 40)
    ground += '\n' + tabstop * (indent+4) + '</plane>'
    ground += '\n' + tabstop * (indent+3) + '</geometry>'
    ground += '\n' + tabstop * (indent+3) + '<material>'
    ground += '\n' + tabstop * (indent+4) + '<script>'
    ground += '\n' + tabstop * (indent+5) + '<uri>model://urban/materials/scripts/</uri>'
    ground += '\n' + tabstop * (indent+5) + '<uri>model://urban/materials/textures/</uri>'
    ground += '\n' + tabstop * (indent+5) + '<name>grass/image</name>'
    ground += '\n' + tabstop * (indent+4) + '</script>'
    ground += '\n' + tabstop * (indent+3) + '</material>'
    ground += '\n' + tabstop * (indent+2) + '</visual>'
    ground += '\n' + tabstop * (indent+2) + '<velocity_decay>'
    ground += '\n' + tabstop * (indent+3) + '<linear>0</linear>'
    ground += '\n' + tabstop * (indent+3) + '<angular>0</angular>'
    ground += '\n' + tabstop * (indent+2) + '</velocity_decay>'
    ground += '\n' + tabstop * (indent+2) + '<self_collide>0</self_collide>'
    ground += '\n' + tabstop * (indent+2) + '<kinematic>0</kinematic>'
    ground += '\n' + tabstop * (indent+2) + '<gravity>1</gravity>'
    ground += '\n' + tabstop * (indent+1) + '</link>'
    ground += '\n' + tabstop * indent + '</model>'
    return ground, vehicle_waypoints

def createLight(pos, diffuse, specular, range, constant, linear, quadratic, direction, indent):
    light = '\n' + tabstop * indent + '<light name="sun" type="directional">'
    light += '\n' + tabstop * (indent+1) + '<cast_shadows>1</cast_shadows>'
    light += '\n' + tabstop * (indent+1) + '<pose>%d %d %d %d %d %d</pose>' % pos
    light += '\n' + tabstop * (indent+1) + '<diffuse>%f %f %f %f</diffuse>' % diffuse
    light += '\n' + tabstop * (indent+1) + '<specular>%f %f %f %f</specular>' % specular
    light += '\n' + tabstop * (indent+1) + '<attenuation>'
    light += '\n' + tabstop * (indent+2) + '<range>%f</range>' % range
    light += '\n' + tabstop * (indent+2) + '<constant>%f</constant>' % constant
    light += '\n' + tabstop * (indent+2) + '<linear>%f</linear>' % linear
    light += '\n' + tabstop * (indent+2) + '<quadratic>%f</quadratic>' % quadratic
    light += '\n' + tabstop * (indent+1) + '</attenuation>'
    light += '\n' + tabstop * (indent+1) + '<direction>%f %f %f</direction>' % direction
    light += '</light>'
    return light

def createPhysics(physics_type, max_step_size, rt_factor, rt_rate, gravity, indent):
    physics = '\n' + tabstop * indent + '<physics type="%s">' % physics_type
    physics += '\n' + tabstop * (indent+1) + '<max_step_size>%f</max_step_size>' % max_step_size
    physics += '\n' + tabstop * (indent+1) + '<real_time_factor>%f</real_time_factor>' % rt_factor
    physics += '\n' + tabstop * (indent+1) + '<real_time_update_rate>%f</real_time_update_rate>' % rt_rate
    physics += '\n' + tabstop * (indent+1) + '<gravity>%f %f %f</gravity>' % gravity
    physics += '\n' + tabstop * indent + '</physics>'
    return physics

def createScene(ambient, background, shadows, indent):
    scene = '\n' + tabstop * indent + '<scene>'
    scene += '\n' + tabstop * (indent+1) + '<ambient>%f %f %f %f</ambient>' % ambient
    scene += '\n' + tabstop * (indent+1) + '<background>%f %f %f %f</background>' % background
    scene += '\n' + tabstop * (indent+1) + '<shadows>%d</shadows>' % shadows
    scene += '\n' + tabstop * indent + '</scene>'
    return scene

def createSphereCoord(surface, latitude, longitude, elevation, heading, indent):
    coord = '\n' + tabstop * indent + '<spherical_coordinates>'
    coord  += '\n' + tabstop * (indent+1) + '<surface_model>%s</surface_model>' % surface
    coord  += '\n' + tabstop * (indent+1) + '<latitude_deg>%f</latitude_deg>' % latitude
    coord  += '\n' + tabstop * (indent+1) + '<longitude_deg>%f</longitude_deg>' % longitude
    coord  += '\n' + tabstop * (indent+1) + '<elevation>%f</elevation>' % elevation
    coord  += '\n' + tabstop * (indent+1) + '<heading_deg>%f</heading_deg>' % heading
    coord  += '\n' + tabstop * indent + '</spherical_coordinates>'
    return coord

def createGui(fullscreen, camera, pose, view, indent):
    gui = '\n' + tabstop * indent + '<gui fullscreen="%d">' % fullscreen
    gui  += '\n' + tabstop * (indent+1) + '<camera name="%s">' % camera
    gui  += '\n' + tabstop * (indent+1) + '<pose>%f %f %f %f %f %f</pose>' % pose
    gui  += '\n' + tabstop * (indent+1) + '<view_controller>%s</view_controller>' % view
    gui  += '\n' + tabstop * (indent+1) + '</camera>'
    gui += '\n' + tabstop * indent + '</gui>'
    return gui

def createWorld(
        light_pos, light_diffuse, light_specular, light_range,
        light_constant, light_linear, light_quadratic, light_direction,
        ground_pos, ground_size, num_roads_x, num_roads_y, num_buildings, road_width, road_height,
        physics_type, physics_step_size, physics_rt_factor, physics_rt_rate, gravity,
        ambient, background, shadows,
        surface, latitude, longitude, elevation, heading,
        fullscreen, camera, gui_pos, view
        ):
    world = '\n' + '<sdf version="1.4">'
    world += '\n' + tabstop + '<world name="default">'
    world += createLight(
        light_pos, light_diffuse, light_specular, light_range, light_constant,
        light_linear, light_quadratic, light_direction, 2)
    ground, vehicle_waypoints = createGround(ground_pos, ground_size, num_roads_x, num_roads_y, num_buildings, road_width, road_height, 2)
    world += ground
    world += createPhysics(physics_type, physics_step_size, physics_rt_factor, physics_rt_rate, gravity, 2)
    world += createScene(ambient, background, shadows, 2)
    world += createSphereCoord(surface, latitude, longitude, elevation, heading, 2)
    world += createGui(fullscreen, camera, gui_pos, view, 2)
    world += '\n' + tabstop + '</world>'
    world += '\n' + '</sdf>'
    return world, vehicle_waypoints

if __name__=="__main__":
    x = 600
    y = 200
    num_vertical_roads = 2
    num_horizontal_roads = 3
    try:
        filename = argv[1]
        size = (int(argv[2]), int(argv[3]))
        num_vertical_roads = int(argv[4])
        num_horizontal_roads = int(argv[5])
        num_buildings_per_space = int(argv[6])
    except:
        filename = "tmp.world"
        size = (640, 320)
        num_vertical_roads = 3
        num_horizontal_roads = 2
        num_buildings_per_space = 6
        print("Correct usage for none default values is: python3 world_builder.py <filename>.world <world_width> <world_length> <num_vertical_roads> num_horizontal_roads> <num_buildings_per_space>")
    # print(createRoad("r1", (100, 0, 0, 0, 0, 0), 20, 620, 0))
    world, vehicle_waypoints = createWorld(
        (0, 0, 500, 0, -0, 0), (.8,.8,.8,1), (.2,.2,.2,1),1000,.9,.01,.001,(-.5,.1,-.9),
        (0,0,0,0,0,-1.5777), size, num_vertical_roads, num_horizontal_roads, num_buildings_per_space, 10, .5,
        'ode', .001, 1, 1000, (0,0,-9.8),
        (0.4, 0.4, 0.4, 1), (0.7, 0.7, 0.7, 1), 0,
        'EARTH_WGS84', 0, 0, 0, 0,
        0, 'user_camera', (3.14796, -7.99288, 40.8348, 1.15202e-17, 1.41964, 0.02019), 'orbit'
    )
    # print(world)
    with open(filename, 'w') as f:
        f.write(world)

    with open("../launch/vehicle_waypoints.yaml", 'w') as f:
        f.write('waypoints:\n%snodes:\n' % tabstop)
        for node in vehicle_waypoints[0]:
            f.write('%s%s- (%f, %f)\n' % (tabstop, tabstop, node[0], node[1]))
        f.write('%sedges:\n' % tabstop)
        for edge in vehicle_waypoints[1]:
            f.write('%s%s- (%d, %d)\n' % (tabstop, tabstop, edge[0], edge[1]))
