from owlready2 import *
from scenariogeneration import xodr
from scenariogeneration import xosc, prettyprint
from scenariogeneration import ScenarioGenerator
import scenariogeneration as SG
import xml.etree.ElementTree as ET
import time
import sys
import os
onto_path.append("./" + sys.argv[1])
wholepath = "file://./" + sys.argv[1] #Change this if you want to import a specific ontology file
template_ontology = get_ontology(wholepath).load()


class Scenario(ScenarioGenerator):
    """
    Class taken from the PYOSCX Library, overwritten for the current purposes.
    """
    def __init__(self):
        ScenarioGenerator.__init__(self)
        self.parameters = []
        current_time = time.localtime()
        d1 = {}
        d1['Scenario'] = "_" + str(current_time.tm_hour) + "_" + str(current_time.tm_min) + "_" + str(
            current_time.tm_sec)
        self.parameters.append(d1)
        self.naming = 'parameter'

    def road(self, **kwargs):
        #Used as part of the template in the PYOSCX Library, a xodr file is being created, but is not used at all, since the used road network is from CARLA
        road = xodr.create_road(
            [xodr.Spiral(0.0000000001, 0.001, 100), xodr.Arc(0.001, 50), xodr.Spiral(0.0001, 0.0000000001, 100),
             xodr.Line(100)], id=0, left_lanes=2, right_lanes=2)
        odr = xodr.OpenDrive('myroad')
        odr.add_road(road)
        odr.adjust_roads_and_lanes()
        return odr

    def scenario(self, **kwargs):
        #Reads the ontology and creates the PYOSCX Scenario accordingly.
        scenario_instances = template_ontology.Scenario.instances()
        if len(scenario_instances) >= 2:
            #If the ontology has more scenario instances, then the following snippet,
            #creates a new scenario individual which is a combination of all Scenario individuals in the given ontology.
            buffer_scenario = template_ontology.Scenario("buffer_scenario")
            main_scenario = template_ontology.Scenario("main_scenario")
            main_init = template_ontology.Init("main_init")
            main_storyboard = template_ontology.Storyboard("main_storyboard")
            for scenario_in_onto in scenario_instances:
                #add all has_entities to one scenario
                for scenario_entities in scenario_in_onto.has_entity:
                    main_scenario.has_entity.append(scenario_entities)
                #add the storyboard
                buffer_scenario.has_storyboard.append(scenario_in_onto.has_storyboard[0])
                for town in scenario_in_onto.has_town:
                    buffer_scenario.has_town.append(town)
            for buffer_storyboard in buffer_scenario.has_storyboard:
                buffer_init = buffer_storyboard.has_init[0]
                for init_actions in buffer_init.has_init_action:
                    #add the init actions in the buffer init to the new main init.
                    main_init.has_init_action.append(init_actions)
                for stories in buffer_storyboard.has_story:
                    #add the stories in the buffer storyboard to the new main storyboard.
                    main_storyboard.has_story.append(stories)   
            main_storyboard.has_init.append(main_init)
            main_scenario.has_storyboard.append(main_storyboard)
            main_scenario.has_town.append(buffer_scenario.has_town[0])
            scenario_name = main_scenario
            
        else: 
            scenario_name = scenario_instances[0]

        scenario_storyboard = scenario_name.has_storyboard[0]  # Storyboard
        entity_list = scenario_name.has_entity

        init = xosc.Init()
        # Add Entities to the scenario
        entities = xosc.Entities() 
        entities = addEntitiesToScenario(entity_list,entities)
        init_action_list = scenario_storyboard.has_init[0].has_init_action
        # INIT PART
        init = InitActionCheck(init_action_list, init) # Add actions to the Init


        return_storyboard_stop_trigger = xosc.EmptyTrigger("stop") #create a default StopTrigger in case there is no stop trigger in the ontology
        if len(scenario_storyboard.has_stop_trigger) != 0:
            storyboard_stop_trigger = scenario_storyboard.has_stop_trigger[0]
            return_storyboard_stop_trigger = addTrigger(storyboard_stop_trigger) #gets the real StopTrigger
        

        ###CAMERA TO EGO ###
        #Add OpenSCENARIO Camera to the default ego vehicle.
        properties = xosc.Properties()
        properties.add_property('module', 'simple_vehicle_control')
        properties.add_property('attach_camera','true')
        ego_vehicles = onto2.EgoVehicle.instances()
        ego_vehicle_name = getNameFromIRI(ego_vehicles[0].iri)
        controller = xosc.Controller('Camera',properties)
        assign_controller = xosc.AssignControllerAction(controller)
        overried_controller = xosc.OverrideControllerValueAction()
        overried_controller.set_brake(False,0)
        overried_controller.set_steeringwheel(False,0)
        overried_controller.set_throttle(False,0)
        overried_controller.set_gear(False,0.0)
        overried_controller.set_clutch(False,0)
        overried_controller.set_parkingbrake(False,0)
        
        controller_action = xosc.ControllerAction(assign_controller,overried_controller)
        
        init.add_init_action(ego_vehicle_name,controller_action)
        
        storyboard = xosc.StoryBoard(init, return_storyboard_stop_trigger)
        #The following for cycles are responsible for correctly creating all of the needed PYOSCX Objects, in order to fill the previously created PYOSCX Storyboard.
        for stories in scenario_storyboard.has_story:
            #This for checks for all of the stories in the given storyboard and creates each PYOSCX Story accordingly.
            story_name = getNameFromIRI(stories.iri)
            my_story = xosc.Story(story_name)
            scenario_acts = stories.has_act
            for act in scenario_acts:
                #This for checks for all of the acts in the current Story and creates the PYOSCX Act accordingly.
                act_name = getNameFromIRI(act.iri)
                act_start_trigger = act.has_start_trigger[0]
                act_stop_trigger = act.has_stop_trigger[0]
                return_act_start_trigger = addTrigger(act_start_trigger)
                return_act_stop_trigger = addTrigger(act_stop_trigger)
                my_act = xosc.Act(act_name, return_act_start_trigger,return_act_stop_trigger)
                for maneuverGroup in act.has_maneuver_group:
                    #This for checks for all of the ManeuverGroups in the current Act and creates the PYOSCX ManeuverGroup accordingly.
                    maneuver_group_OS = xosc.ManeuverGroup(getNameFromIRI(maneuverGroup.iri))
                    maneuver_entity_ref = getNameFromIRI(maneuverGroup.has_entity_ref[0].iri)
                    for maneuver in maneuverGroup.has_maneuver:
                        #This for checks for all of the Maneuvers in the current ManeuverGroup and creates the PYOSCX Maneuver accordingly.
                        man = xosc.Maneuver(getNameFromIRI(maneuver.iri))
                        for event_in_onto in maneuver.has_event:
                            #This for checks for all of the Events in the current Maneuver and creates the PYOSCX Event accordingly.
                            event_priority = getNameFromIRI(event_in_onto.has_priority[0].iri)
                            #Check the priority of the ontology Event and create the PYOSCX Event accordingly
                            if event_priority == "overwrite":
                                event_to_add = xosc.Event(getNameFromIRI(event_in_onto.iri), xosc.Priority.overwrite)
                            if event_priority == "parallel":
                                event_to_add = xosc.Event(getNameFromIRI(event_in_onto.iri), xosc.Priority.parallel)
                            if event_priority == "skip":
                                event_to_add = xosc.Event(getNameFromIRI(event_in_onto.iri), xosc.Priority.skip)
                            for event_action in event_in_onto.has_action:
                                #This for checks for all of the actions in the current event and creates them accordingly.
                                event_action_name = getNameFromIRI(event_action.iri)  # Name of the action
                                event_action_class = getNameFromIRI(event_action.is_a[0].iri)
                                if event_action_class == "SpeedAction":
                                    target_speed = event_action.has_target_speed[0]
                                    ontology_transition_dynamics = event_action.has_transition_dynamics[0]
                                    xosc_transition_dynamics = checkTransitionDynamics(ontology_transition_dynamics)
                                    event_to_add.add_action(event_action_name, xosc.AbsoluteSpeedAction(target_speed,xosc_transition_dynamics))
                                if event_action_class == "TeleportAction":
                                    # if the action has position as a property assertion
                                    if len(event_action.has_position) != 0:
                                        position = event_action.has_position[0]
                                        if getNameFromIRI(position.is_a[0].iri) == "Position":
                                            s = 0
                                            offset = 0
                                            lane_id = 0
                                            road_id = 0

                                            if len(position.has_s) != 0:
                                                s = position.has_s[0]

                                            if len(position.has_offset) != 0:
                                                offset = position.has_offset[0]

                                            if len(position.has_lane_id) != 0:
                                                lane_id = position.has_lane_id[0]

                                            if len(position.has_road_id) != 0:
                                                road_id = position.has_road_id[0]
                                            event_to_add.add_action(event_action_name, xosc.TeleportAction(xosc.LanePosition(s, offset, lane_id, road_id))) #Create PYOSCX TeleportAction according to the given object property assertions

                                        if getNameFromIRI(position.is_a[0].iri) == "RelativeObjectPosition":
                                            relpos_entity_ref = getNameFromIRI(
                                                position.has_relative_position_object_reference[0].iri)
                                            x, y, z = 0, 0, 0
                                            if len(position.has_x) != 0:
                                                x = position.has_x[0]
                                            if len(position.has_y) != 0:
                                                y = position.has_y[0]
                                            if len(position.has_z) != 0:
                                                z = position.has_z[0]
                                            if len(position.has_orientation) != 0:
                                                orientation = position.has_orientation[0]
                                                pitch, roll, heading = 0, 0, 0
                                                if len(orientation.has_pitch) != 0:
                                                    pitch = orientation.has_pitch[0]
                                                if len(orientation.has_roll) != 0:
                                                    roll = orientation.has_roll[0]
                                                if len(orientation.has_heading) != 0:
                                                    heading = orientation.has_heading[0]
                                                event_to_add.add_action(event_action_name, xosc.TeleportAction(
                                                    xosc.RelativeObjectPosition(relpos_entity_ref, x, y, z,
                                                                                xosc.Orientation(heading, pitch, roll)))) #create a PYOSCX TeleportAction with RelativeObjectPosition
                                            else:
                                                event_to_add.add_action(event_action_name, xosc.TeleportAction(
                                                    xosc.RelativeObjectPosition(relpos_entity_ref, x, y, z)))

                                if event_action_class == "EnvironmentAction":
                                    ea = checkEnvironmentAction(event_action) #Create a PYOSCX EnvironmentAction
                                    event_to_add.add_action(event_action_name,ea) 
                                if event_action_class == "RelativeLaneChangeAction":
                                    rel_lane_id = 0
                                    rel_offset = 0
                                    if len(event_action.has_target_lane_id) != 0 :
                                        rel_lane_id = event_action.has_target_lane_id[0] 
                                    if len(event_action.has_offset) !=0:
                                        rel_offset = event_action.has_offset[0]
                                    if len(event_action.has_entity_ref) != 0:
                                        rel_entity_ref = getNameFromIRI(event_action.has_entity_ref[0].iri)
                                    if len(event_action.has_transition_dynamics) != 0:
                                        onto_transition_dynamics = event_action.has_transition_dynamics[0]
                                        xosc_transition_dyn = checkTransitionDynamics(onto_transition_dynamics)
                                    event_to_add.add_action(event_action_name,xosc.RelativeLaneChangeAction(rel_lane_id,rel_entity_ref,xosc_transition_dyn,rel_offset)) #Create a PYOSCX RelativeLaneChangeAction
                                if len(event_in_onto.has_start_trigger) !=0 :
                                    event_trigger = addTrigger(event_in_onto.has_start_trigger[0])
                                    event_to_add.add_trigger(event_trigger)                #Create a PYOSCX StartTrigger and add it to the PYOSCX Event
                            man.add_event(event_to_add) #Add PYOSCX Event to PYOSCX Maneuver
                    maneuver_group_OS.add_maneuver(man) #Add PYOSCX Maneuver to PYOSCX ManeuverGroup
                    maneuver_group_OS.add_actor(maneuver_entity_ref) #Add entity reference to PYOSCX ManeuverGroup
                    my_act.add_maneuver_group(maneuver_group_OS)
                my_story.add_act(my_act) #Add PYOSCX Act to PYOSCX Story
            storyboard.add_story(my_story) #Add PYOSCX Story to PYOSCX Storyboard 
        town = getNameFromIRI(scenario_name.has_town[0].iri)
        road = xosc.RoadNetwork(roadfile=town,scenegraph="")
        catalog = xosc.Catalog()
        scenario = xosc.Scenario(getNameFromIRI(scenario_name.iri), 'Stefani', xosc.ParameterDeclarations(), entities, storyboard, road, catalog, 0) #Create the PYOSCX Scenario with Rev Minor 0
        return scenario


def checkEnvironmentAction(ontology_environment_action):
    """
    Creates a PYOSCX EnvironmentAction object, according to the given in the ontology individual property assertions.

    Parameters:

    ontology_environment_action - an ontology individual of the class EnvironmentAction

    Returns a reference to the new PYOSCX EnvironmentAction
    """
    name = getNameFromIRI(ontology_environment_action.iri)
    environment = ontology_environment_action.has_environment[0]
    xosc_environment = checkEnvironment(environment)
    return xosc.EnvironmentAction(xosc_environment)

def checkDynamicsDimension(dynamics_dimension):
    """
    According to the given in the ontology individual property assertion, returns the correct PYOSCX DynamicsDimension Enumeration.

    Parameters:

    dynamics_dimension - an ontology individual of the class DynamicsDimension
    """
    xosc_dynamics_dimension = xosc.DynamicsDimension.distance
    if dynamics_dimension == "distance":
        xosc_dynamics_dimension = xosc.DynamicsDimension.distance
    if dynamics_dimension == "rate":
        xosc_dynamics_dimension = xosc.DynamicsDimension.rate 
    if dynamics_dimension == "time":
        xosc_dynamics_dimension = xosc.DynamicsDimension.time
    return xosc_dynamics_dimension
    
    
def checkDynamicsShapes(dynamics_shape):
    """
    According to the given in the ontology individual property assertion, Returns the correct PYOSCX DynamicsShape Enumeration.

    Parameters:

    dynamics_shape - an ontology individual of the class DynamicsShape
    """
    xosc_dynamics_shape = xosc.DynamicsShapes.step
    if dynamics_shape == "cubic":
        xosc_dynamics_shape = xosc.DynamicsShapes.cubic
    if dynamics_shape == "cubic":
        xosc_dynamics_shape = xosc.DynamicsShapes.linear
    if dynamics_shape == "cubic":
        xosc_dynamics_shape = xosc.DynamicsShapes.step
    if dynamics_shape == "sinusoidal":
        xosc_dynamics_shape = xosc.DynamicsShapes.sinusoidal   
    return xosc_dynamics_shape
    
    
def checkTransitionDynamics(ontology_transition_dynamics):
    """
    Creates a PYOSCX TransitionDynamics object, according to the given in the ontology individual property assertions.

    Parameters:

    ontology_transition_dynamics - an ontology individual of the class TransitionDynamics

    Returns a reference to the new PYOSCX TransitionDynamics
    """
    ontology_dynamics_dimension = getNameFromIRI(ontology_transition_dynamics.has_dynamics_dimension[0].iri)
    ontology_dynamics_shape = getNameFromIRI(ontology_transition_dynamics.has_dynamics_shape[0].iri)
    xosc_dynamics_shape = checkDynamicsShapes(ontology_dynamics_shape)
    xosc_dynamics_dimension = checkDynamicsDimension(ontology_dynamics_dimension)
    value = ontology_transition_dynamics.has_value[0]
    return xosc.TransitionDynamics(xosc_dynamics_shape,xosc_dynamics_dimension,value)
    
    
def checkEnvironment(ontology_environment):
    """
    Creates a PYOSCX Environment, TimeOfDay, Weather and RoadCondition objects, according to the given in the ontology individual property assertions.

    Parameters:

    ontology_environment - an ontology individual of the class Environment

    Returns a reference to the new PYOSCX Environment object
    """
    ontology_time_of_day = ontology_environment.has_time_of_day[0] #get the TimeOfDay individual in the ontology 
    #Check TimeOfDay property assertions in the ontology and create the PYOSCX TimeOfDay accordingly.
    if len(ontology_time_of_day.has_animation) != 0:
        animation = ontology_time_of_day.has_animation[0]
    if len(ontology_time_of_day.has_year) != 0:
        year = ontology_time_of_day.has_year[0]
    if len(ontology_time_of_day.has_month) != 0:
        month = ontology_time_of_day.has_month[0]        
    if len(ontology_time_of_day.has_day) != 0:
        day = ontology_time_of_day.has_day[0]    
    if len(ontology_time_of_day.has_hour) != 0:
        hour = ontology_time_of_day.has_hour[0]
    if len(ontology_time_of_day.has_minute) != 0:
        minute = ontology_time_of_day.has_minute[0]
    if len(ontology_time_of_day.has_second) != 0:
        second = ontology_time_of_day.has_second[0]
    xosc_time_of_day = xosc.TimeOfDay(animation,year,month,day,hour,minute,second)
    #Check Weather property assertions in the ontology and create the PYOSCX Weather accordingly.
    ontology_weather = ontology_environment.has_weather[0] #get the Weather individual in the ontology
    if len(ontology_weather.has_cloud_state) != 0:
        xosc_cloud_state = checkCloudState(ontology_weather.has_cloud_state[0])
    if len(ontology_weather.has_fog) !=0:
        xosc_fog = checkFog(ontology_weather.has_fog[0])
    if len(ontology_weather.has_sun) !=0:
        xosc_sun = checkSun(ontology_weather.has_sun[0])
    if len(ontology_weather.has_precipitation) !=0:
        xosc_precipitation = checkPrecipitation(ontology_weather.has_precipitation[0])
    xosc_weather = xosc.Weather(xosc_cloud_state,sun = xosc_sun, fog = xosc_fog, precipitation = xosc_precipitation)
    #Check RoadCondtion property assertions in the ontology and create the PYOSCX RoadCondition accordingly.
    ontology_road_condition = ontology_environment.has_road_condition[0] #get the RoadCondition individual in the ontology
    if len(ontology_road_condition.has_friction_scale_factor) !=0:
        friction_scale_factor = ontology_road_condition.has_friction_scale_factor[0]
        xosc_road_condition = xosc.RoadCondition(friction_scale_factor)
    environment_name = getNameFromIRI(ontology_environment.iri)
    return xosc.Environment(environment_name,xosc_time_of_day,xosc_weather,xosc_road_condition)

def checkPrecipitation(ontology_precipitation):
    """
    Creates a PYOSCX Precipitation object, according to the given in the ontology individual property assertions.

    Parameters:

    ontology_precipitation - an ontology individual of the class Precipitation.

    Returns a reference to the new PYOSCX Precipitation object according to the precipitation type.
    """
    precipitation_type = getNameFromIRI(ontology_precipitation.has_precipitation_type[0].iri) #gets the chosen precipitation type constant individual in the ontology
    intensity = ontology_precipitation.has_intensity[0] 
    if precipitation_type == "dry":
        return xosc.Precipitation(xosc.PrecipitationType.dry,intensity)
    if precipitation_type == "rain":
        return xosc.Precipitation(xosc.PrecipitationType.rain,intensity)
    if precipitation_type == "snow":
        return xosc.Precipitation(xosc.PrecipitationType.snow,intensity)    


def checkSun(ontology_sun):
    """
    Creates a PYOSCX Sun object, according to the given in the ontology individual property assertions.

    Parameters:

    ontology_sun - an ontology individual of the class Sun.

    Returns a reference to the new PYOSCX Sun object
    """
    elevation = ontology_sun.has_elevation[0] #gets the elevation value of the Sun in the ontology. 
    azimuth = ontology_sun.has_azimuth[0] #gets the azimuth value of the Sun in the ontology. 
    intensity = ontology_sun.has_intensity[0] #gets the intensity value of the Sun in the ontology.
    return xosc.Sun(intensity,azimuth,elevation)

def checkFog(ontology_fog):
    """
    Creates a PYOSCX Fog object, according to the given in the ontology individual property assertions.

    Parameters:

    ontology_fog - an ontology individual of the class Fog.

    Returns a reference to the new PYOSCX Fog object
    """
    if len(ontology_fog.has_bounding_box) != 0:
        bounding_box = checkBoundingBox(ontology_fog.has_bounding_box[0]) #checks the ontology BoundingBox individual and assigns a correct PYOSCX BoundingBox object.
    if len(ontology_fog.has_visual_range) !=0:
        visual_range = ontology_fog.has_visual_range[0]
    return xosc.Fog(visual_range,bounding_box)

def checkBoundingBox(ontology_bounding_box):
    """
    Creates a PYOSCX BoundingBox object, according to the given in the ontology individual property assertions.

    Parameters:

    ontology_bounding_box - an ontology individual of the class BoundingBox.

    Returns a reference to the new PYOSCX BoundingBox object
    """
    heigth = ontology_bounding_box.has_heigth[0]
    width = ontology_bounding_box.has_width[0]
    length = ontology_bounding_box.has_length[0]
    x = ontology_bounding_box.has_x_center[0]
    y = ontology_bounding_box.has_y_center[0]
    z = ontology_bounding_box.has_z_center[0]
    return xosc.BoundingBox(width,length,heigth,x,y,z)


def main():
    s = Scenario()
    s1 = s.generate('newScenarios')
#    command = "python3 ~/scenario_runner/scenario_runner.py --openscenario ./" + s1[0][0] # In order to run this directly  change it to your scenario_runner folder
#    os.system("python3 ~/scenario_runner/scenario_runner.py --openscenario ./" + s1[0][0])

def getNameFromIRI(iri: str):
    """
    Gets the real name in the ontology of the given object, instead of its Id, so the class or directly the name of the individuals can be used.
    
    Parameters:

    iri - the iri of the ontology object. Get this by calling ".iri" on the individual.

    Returns a string with the name/class of the individual.
    """
    name: str = iri.rsplit('#', 1)[-1]
    assert name.find('/') == -1
    return name



def addEntitiesToScenario(entityList,entities):
    """
    Creates new PYOSCX Entities for every object in the entityList

    Parameters:

    entityList - array with the individuals which have "has_entity" property assertion connected to the Scenario in the ontology.

    entities - a PYOSCX Entities object, to which the new PYOSCX Entities are added.

    Returns an array with PYOSCX Entities
    """
    for entity in entityList:
        entity_bounding_box = xosc.BoundingBox(2,5,1.6,2,0,0.9)
        if len(entity.has_bounding_box) != 0:
            entity_bounding_box = checkBoundingBox(entity.has_bounding_box[0])
        entity_name = getNameFromIRI(entity.iri)
        asset_name = getNameFromIRI(entity.has_asset[0].iri)
        asset_name = asset_name.replace("_",".")
        # is_a - Returns the class of the Individual
        if getNameFromIRI(entity.is_a[0].iri) == "Vehicle":
            fa = xosc.Axle(0.523598775598,0.8,1.68,2.98,0.4) #default example Axles
            ba = xosc.Axle(0.523598775598,0.8,1.68,0,0.4) #default example Axles
            vehicle = xosc.Vehicle(asset_name,xosc.VehicleCategory.car,entity_bounding_box,fa,ba,69,10,200)
            vehicle.add_property('type','simulation')
            entities.add_scenario_object(entity_name,vehicle)
            continue
        if getNameFromIRI(entity.is_a[0].iri) == "EgoVehicle":
            fa = xosc.Axle(0.523598775598,0.8,1.68,2.98,0.4) #default example Axles
            ba = xosc.Axle(0.523598775598,0.8,1.68,0,0.4) #default example Axles
            vehicle = xosc.Vehicle(asset_name,xosc.VehicleCategory.car,entity_bounding_box,fa,ba,69,10,200)
            vehicle.add_property('type','ego_vehicle')
            entities.add_scenario_object(entity_name,vehicle)
            continue
        if getNameFromIRI(entity.is_a[0].iri) == "Bicycle":
            fa = xosc.Axle(0.523598775598,0.8,1.68,2.98,0.4) #default example Axles
            ba = xosc.Axle(0.523598775598,0.8,1.68,0,0.4) #default example Axles
            vehicle = xosc.Vehicle(asset_name,xosc.VehicleCategory.bicycle,entity_bounding_box,fa,ba,69,10,200)
            vehicle.add_property('type','simulation')
            entities.add_scenario_object(entity_name,vehicle)
            continue
        if getNameFromIRI(entity.is_a[0].iri) == "Pedestrian":
            vehicle = xosc.Pedestrian(asset_name,100,xosc.PedestrianCategory.pedestrian,entity_bounding_box,asset_name)
            vehicle.add_property('type','simulation')
            entities.add_scenario_object(entity_name, vehicle)
            continue
        if getNameFromIRI(entity.is_a[0].iri) == "Misc":
            bb = xosc.BoundingBox(2,5,1.5,1.5,0,0.2)
            obj = xosc.MiscObject(asset_name,100,xosc.MiscObjectCategory.obstacle,entity_bounding_box)
            obj.add_property('type','simulation')
            entities.add_scenario_object(entity_name, obj)       
    return entities


def InitActionCheck(initActionList, init):
    """
    Creates and adds PYOSCX Actions according to the ontology actions given in the initActionList

    Parameters:

    initActionList - array with the individuals which have "has_init_action" property assertion connected to the Init in the ontology.

    entities - a PYOSCX Entities object, to which the new PYOSCX Entities are added.

    Returns an PYOSCX Init object with connections to the new PYOSCX Action objects.
    """
    for actions in initActionList:
            action_class = getNameFromIRI(actions.is_a[0].iri)
            # if the action is a SpeedAction class
            if action_class == "SpeedAction":
                action_entity_ref = getNameFromIRI(actions.has_entity_ref[0].iri)
                target_speed = actions.has_target_speed[0]
                ontology_transition_dynamics = actions.has_transition_dynamics[0]
                xosc_transition_dynamics = checkTransitionDynamics(ontology_transition_dynamics)
                init.add_init_action(action_entity_ref, xosc.AbsoluteSpeedAction(target_speed, xosc_transition_dynamics))
                continue
            #if the action is TeleportAction
            if action_class == "TeleportAction":
                action_entity_ref = getNameFromIRI(actions.has_entity_ref[0].iri)
                # if the action has position as parameter set
                s: int = 0
                offset = 0
                lane_id = 0
                road_id = 0
                if len(actions.has_position) != 0:
                    position = actions.has_position[0]
                    if len(position.has_s) != 0:
                        s = position.has_s[0]

                    if len(position.has_offset) != 0:
                        offset = position.has_offset[0]

                    if len(position.has_lane_id) != 0:
                        lane_id = position.has_lane_id[0]

                    if len(position.has_road_id) != 0:
                        road_id = position.has_road_id[0]

                init.add_init_action(action_entity_ref, xosc.TeleportAction(xosc.LanePosition(s, offset, lane_id, road_id)))
                continue
            if action_class == "EnvironmentAction": # if the action is an EnvironmentAction
                xosc_environment_action = checkEnvironmentAction(actions)
                init.add_global_action(xosc_environment_action)
    return init


def addTrigger(trigger):
    """
    Creates PYOSCX StartTrigger/StopTrigger according to the given property assertions in the ontology.

    Creates PYOSCX Condition and ConditionGroup accordingly.
    Parameters:

    initActionList - array with the individuals which have "has_init_action" property assertion connected to the Init in the ontology.

    entities - a PYOSCX Entities object, to which the new PYOSCX Entities are added.

    Returns an PYOSCX StartTrigger/StopTrigger
    """
    trigger_type = ""
    simulation_time = 0
    delay = 0
    trigger_class = getNameFromIRI(trigger.is_a[0].iri)
    if trigger_class == "StartTrigger":
        return_trigger = xosc.Trigger("start")
    if trigger_class == "StopTrigger":
        return_trigger = xosc.Trigger("stop")
    trigger_condition_group = trigger.has_condition_group[0] #gets the ConditionGroup individual
    condition_group = xosc.ConditionGroup()
    trigger_name = getNameFromIRI(trigger.iri)
    for condition in trigger_condition_group.has_condition:
        condition_class = getNameFromIRI(condition.is_a[0].iri)
        #SimulationTimeCondtion
        if condition_class == "SimulationTimeCondition":
            if len(condition.has_simulation_time_condition) != 0:
                simulation_time = condition.has_simulation_time_condition[0]
                                                       
            if len(condition.has_condition_rule) != 0:
                condition_rule_value = getNameFromIRI(condition.has_condition_rule[0].iri)
                xosc_condition_rule = conditionRule(condition_rule_value)                                                            
            condition_for_the_trigger = xosc.SimulationTimeCondition(simulation_time,xosc_condition_rule)
            trigger_type = "Value"
        #StoryboardElementCondition
        if condition_class == "StoryboardElementStateCondition":
            if len(condition.has_state) != 0:
                condition_state = getNameFromIRI(condition.has_state[0].iri)
                if condition_state == "completeState":
                    xosc_condition_state = xosc.StoryboardElementState.completeState
                if condition_state == "runningState":
                    xosc_condition_state = xosc.StoryboardElementState.runningState
                if condition_state == "standbyState":
                    xosc_condition_state = xosc.StoryboardElementState.standbyState
                if condition_state == "startTransition":
                    xosc_condition_state = xosc.StoryboardElementState.startTransition
                if condition_state == "stopTransition":
                    xosc_condition_state = xosc.StoryboardElementState.stopTransition
                if condition_state == "skipTransition":
                    xosc_condition_state = xosc.StoryboardElementState.skipTransition
                if condition_state == "endTransition":
                    xosc_condition_state = xosc.StoryboardElementState.endTransition
                #check the class of the element ref and then derive the element type
            if len(condition.has_storyboard_element_ref) != 0:
                element_ref_name = getNameFromIRI(condition.has_storyboard_element_ref[0].iri)
                element_ref_class = getNameFromIRI(condition.has_storyboard_element_ref[0].is_a[0].iri) #instead of has_storyboard_element_type
                if element_ref_class == "Story":
                    xosc_element = xosc.StoryboardElementType.story
                if element_ref_class == "Act":
                    xosc_element = xosc.StoryboardElementType.act
                if element_ref_class == "Maneuver":
                    xosc_element = xosc.StoryboardElementType.maneuver
                if element_ref_class == "Event":
                    xosc_element = xosc.StoryboardElementType.event
                if element_ref_class == "Action":
                    xosc_element = xosc.StoryboardElementType.action
                if element_ref_class == "ManeuverGroup":
                    xosc_element = xosc.StoryboardElementType.maneuverGroup 
            condition_for_the_trigger = xosc.StoryboardElementStateCondition(xosc_element,element_ref_name,xosc_condition_state)
            trigger_type = "Value"
        #RelativeDistanceCondtion
        if condition_class == "RelativeDistanceCondition":
            relative_distance_type = xosc.RelativeDistanceType.lateral
            if len(condition.has_relative_distance_type) != 0:
                distance_type_value = getNameFromIRI(condition.has_relative_distance_type[0].iri)
                if distance_type_value == "cartesianDistance":
                    relative_distance_type = xosc.RelativeDistanceType.cartesianDistance
                if distance_type_value == "lateral":
                    relative_distance_type = xosc.RelativeDistanceType.lateral
                if distance_type_value == "longitudinal":
                    relative_distance_type = xosc.RelativeDistanceType.longitudinal
            if len(condition.has_condition_rule) != 0:
                condition_rule_value = getNameFromIRI(condition.has_condition_rule[0].iri)
                xosc_condition_rule = conditionRule(condition_rule_value)  
            if len(condition.has_target_reference) != 0:
                target_ref_name = getNameFromIRI(condition.has_target_reference[0].iri)
            if len(condition.has_distance_value) != 0:
                distance_value = condition.has_distance_value[0]
            condition_for_the_trigger = xosc.RelativeDistanceCondition(distance_value,xosc_condition_rule,relative_distance_type,target_ref_name, freespace = False)
            trigger_type = "Entity"
        #SpeedCondition
        if condition_class == "SpeedCondition":
            trigger_type = "Entity"
            speed = condition.has_condition_speed_value[0]
            xosc_condition_rule = conditionRule(getNameFromIRI(condition.has_condition_rule[0].iri))
            condition_for_the_trigger = xosc.SpeedCondition(speed,xosc_condition_rule)
        if condition_class == "TraveledDistanceCondition":
            trigger_type = "Entity"
            distance = condition.has_traveled_distance_value[0]
            condition_for_the_trigger = xosc.TraveledDistanceCondition(distance)
        #Parameters for every trigger
        #Condition Edge of the Condition
        xosc_condition_edge = xosc.ConditionEdge.none
        if len(condition.has_condition_edge) !=0:
            condition_edge_value = getNameFromIRI(condition.has_condition_edge[0].iri)
            if condition_edge_value == "rising":
                xosc_condition_edge = xosc.ConditionEdge.rising
            if condition_edge_value == "falling": 
                xosc_condition_edge = xosc.ConditionEdge.falling
            if condition_edge_value == "none":
                xosc_condition_edge = xosc.ConditionEdge.none
            if condition_edge_value == "risingOrFalling":
                xosc_condition_edge = xosc.ConditionEdge.risingOrFalling
        #Delay of the condition
        condition_name = getNameFromIRI(condition.iri)
        if len(condition.has_delay) != 0:
                delay = condition.has_delay[0]
        if(trigger_type == "Value"):
            value_trigger = xosc.ValueTrigger(condition_name,delay,xosc_condition_edge,condition_for_the_trigger)
            condition_group.add_condition(value_trigger)
        if(trigger_type == "Entity"):
            if len(condition.has_entity_ref) !=0:
                entity_ref = getNameFromIRI(condition.has_entity_ref[0].iri)
            entity_trigger = xosc.EntityTrigger(condition_name,delay,xosc_condition_edge,condition_for_the_trigger,entity_ref)
            condition_group.add_condition(entity_trigger)
    return_trigger.add_conditiongroup(condition_group)
    return return_trigger
    
def checkCloudState(ontology_cloud_state):
    """
    According to the given in the ontology individual property assertion, Returns the correct PYOSCX Enumeration for CloudState.

    Parameters:

    ontology_cloud_state - an ontology individual of the class CloudState
    """
    xosc_cloud_state = xosc.CloudState.free
    if ontology_cloud_state == "skyOff":
        xosc_cloud_state = xosc.CloudState.skyOff
    if ontology_cloud_state == "free":    
        xosc_cloud_state = xosc.CloudState.free
    if ontology_cloud_state == "cloudy":    
        xosc_cloud_state = xosc.CloudState.cloudy
    if ontology_cloud_state == "overcast":
        xosc_cloud_state = xosc.CloudState.overcast
    if ontology_cloud_state == "rainy":   
        xosc_cloud_state = xosc.CloudState.rainy
    return xosc_cloud_state


def conditionRule(condition_rule_value):
    """
    According to the given in the ontology individual property assertion, Returns the correct PYOSCX Enumeration for ConditionRule.

    Parameters:

    condition_rule_value - an ontology individual of the class ConditionRule
    """
    if condition_rule_value == "greaterThan":
        xosc_condition_rule = xosc.Rule.greaterThan
    if condition_rule_value == "lessThan":
        xosc_condition_rule = xosc.Rule.lessThan
    if condition_rule_value == "equalTo":
        xosc_condition_rule = xosc.Rule.equalTo
    if condition_rule_value == "greaterOrEqual":
        xosc_condition_rule = xosc.Rule.greaterOrEqual
    if condition_rule_value == "lessOrEqual":
        xosc_condition_rule = xosc.Rule.lessOrEqual
    if condition_rule_value == "notEqualTo":
        xosc_condition_rule = xosc.Rule.notEqualTo    
    return xosc_condition_rule

    
if __name__ == "__main__":
    main()
 
