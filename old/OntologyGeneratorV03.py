from mimetypes import init
from owlready2 import *
import time
onto_path.append("/disk/users/xt274/Stefani/phd/generation")
onto2 = get_ontology("file:///fzi/ids/xt274/no_backup/phd/generation/MainOntologyV05.owl").load()
#onto_path.append("/Users/qala/Desktop/test_swrl_bachelor")
#onto2 = get_ontology("file:///Users/qala/Desktop/test_swrl_bachelor/OpenScenarioMainOntology_v02.owl").load()
import os

def newSpeedAction(entity, target_speed,transition_dynamics, n1, n2):
    """
    Creates an individual of class SpeedAction in the new ontology with name "n1n2SpeedAction" 

    Parameters:

    entity - an individual with the subclasses of the Class Entity. This is the entity reference of the SpeedAction.
    Use newVehicle(), newBicycle(), newPedestrian() or newEgoVehicle() for  this.

    target_speed(int) - target speed of the speed action.

    transition_dynamics - a transition dynamics individual of the class TransitionDynamics, created with the method newTransitionDynamics()

    n1, n2 - string used for the name of the individual

    Returns a reference to the new SpeedAction individual within the ontology
    """
    action = onto2.SpeedAction(n1 + n2 + "SpeedAction" )
    action.has_entity_ref.append(entity)
    action.has_target_speed.append(target_speed)
    action.has_transition_dynamics.append(transition_dynamics) 
    return action


def getBicycleAssets():
    """
    Returns an array with cyclist assets. The given strings are created as an individual in the ontology.    
    """
    cyclists = ["vehicle.diamondback.century"]
    return cyclists

def  getCarAssets():
    """
    Returns an array with vehicle assets. The given strings are created as an individual in the ontology.    
    """
    cars = ["vehicle_volkswagen_t2", "vehicle_tesla_model3","vehicle.ford.mustang","vehicle.kawasaki.ninja"]
    return cars
    
    
def getPedestrianAssets():
    """
    Returns an array with pedestrian assets. The given strings are created as an individual in the ontology.    
    """
    pedestrians = ["walker_pedestrian_0001","walker_pedestrian_0002","walker_pedestrian_0003","walker_pedestrian_0004","walker_pedestrian_0005","walker_pedestrian_0006","walker_pedestrian_0007","walker_pedestrian_0008","walker_pedestrian_0009","walker_pedestrian_0010","walker_pedestrian_0011"]
    return pedestrians
    

def newTeleportActionWithPosition(entity, lane_id, s, offset, road_id, n1, n2):
    """
    Creates an individual of class TeleportAction in the new ontology with name "n1n2TeleportAction". 
    
    Creates an individual of class Position in the ontology.

    Parameters:

    entity - An individual with the subclasses of the Class Entity. This is the entity reference of the TeleportAction.
    Use newVehicle(), newBicycle(), newPedestrian(), newEgoVehicle() or newMisc() to create this.

    lane_id(int) - ID of the current lane (ID of a lane in road network).

    s(int) - Represents s coordinate along the reference line of the road.

    offset - Lateral offset to the centerline of the current lane

    road_id(int) - ID of the current road (ID of a road in road network). 

    n1, n2 - string used for the name of the individual

    Returns a reference to the new TeleportAction individual within the ontology
    """
    action = onto2.TeleportAction(n1 + n2 + "TeleportAction")
    position = onto2.Position(n1 + n2 + "Position")
    action.has_position.append(position)
    position.has_lane_id.append(lane_id)
    position.has_s.append(s)
    position.has_offset.append(offset)
    position.has_road_id.append(road_id)
    action.has_entity_ref.append(entity)
    return action

def newTeleportActionWithRelativePositionAndOrientation(n1,n2,obj_ref,x,y,z,pitch,roll,heading):
    """
    Creates an individual of class TeleportAction in the new ontology with name "n1n2TeleportAction". 
    
    Creates an individual of class RelativeObjectPosition in the ontology.

    Creates an individual of class Orientation in the ontology.

    Parameters:

    n1, n2 - string used for the name of the individual 

    obj_ref - An individual with the subclasses of the Class Entity. This is the entity reference of the TeleportAction.
    Use newVehicle(), newBicycle(), newPedestrian(), newEgoVehicle() or newMisc() to create this.

    x, y, z(double) - Relative position in the x, y, z  axis, using the coordinate system of the reference entity.
    
    pitch(double) - Pitch angle. Unit: rad
    
    roll(double) - Roll angle. Unit: rad
    
    heading(double) - Heading angle. Unit: rad 

    Returns a reference to the new TeleportAction individual within the ontology
    """
    action = onto2.TeleportAction(n1+n2+"TeleportActionRelative")
    position = onto2.RelativeObjectPosition(n1+n2+"RelativeObjectPosition")
    position.has_relative_position_object_reference.append(obj_ref)
    position.has_x.append(x)
    position.has_y.append(y)
    position.has_z.append(z)
    orientation = onto2.Orientation(n1+n2+"Orientation")
    orientation.has_pitch.append(pitch)
    orientation.has_heading.append(heading)
    orientation.has_roll.append(roll)
    position.has_orientation.append(orientation)
    action.has_position.append(position)
    return action
    
def newManeuver(events, n1, n2):
    """
    Creates an individual of class Maneuver in the new ontology with name "n1n2Maneuver".
    Parameters:

    events - an array with individuals of class Event. Use newEvent() to create those.

    n1, n2 - string used for the name of the individual 

    Returns a reference to the new Maneuver individual within the ontology
    """
    maneuver = onto2.Maneuver(n1 + n2 + "Maneuver")
    for event in events:
        #add the current event as property assertion to the manuever.
        maneuver.has_event.append(event)
    return maneuver


def newManeuverGroup(maneuvers, entity, n1, n2):
    """
    Creates an individual of class ManeuverGroup in the new ontology with name "n1n2ManeuverGroup". 

    Parameters:

    maneuvers - an array with individuals of class Maneuver. Use newManeuver() to create those.

    entity - an entity reference. This entity then executes all maneuvers. Use newVehicle(), newBicycle(), newPedestrian(), newEgoVehicle() or newMisc() to create this.
    
    n1, n2 - string used for the name of the individual 

    Returns a reference to the new Maneuver individual within the ontology
    """
    mnvGrp = onto2.ManeuverGroup(n1 + n2 + "ManeuverGroup")
    mnvGrp.has_entity_ref.append(entity)
    for maneuver in maneuvers:
        #add the current maneuver as property assertion to the manuever group.
        mnvGrp.has_maneuver.append(maneuver)
    return mnvGrp

def newAct(maneuver_groups,start_trigger,stop_trigger,n1,n2):
    """
    Creates an individual of class Act in the new ontology with name "n1n2Act". 

    Parameters:

    maneuver_groups - an array with individuals of class ManeuverGroup. Use newManeuverGroup() to create those.

    start_trigger - an individual of the class StartTrigger. Use newStartTrigger() to create those. 

    stop_trigger - an individual of the class StopTrigger. Use newStopTrigger() to create those. 

    n1, n2 - string used for the name of the individual 

    Returns a reference to the new Act individual within the ontology
    """
    act = onto2.Act(n1 + n2+"Act")
    for mg in maneuver_groups:
        #add the current maneuver group(mg) as property assertion to the act.
        act.has_maneuver_group.append(mg)
    act.has_stop_trigger.append(stop_trigger) # add the stop trigger to the act.
    act.has_start_trigger.append(start_trigger) # add the start trigger to the act.
    return act
    
def newStory(acts,stop_trigger,n1,n2):
    """
    Creates an individual of class Story in the new ontology with name "n1n2Story". 

    Parameters:

    acts - an array with individuals of class Act. Use newAct() to create those.

    stop_trigger - an individual of the class StopTrigger. Use newStopTrigger() to create those. 

    n1, n2 - string used for the name of the individual 

    Returns a reference to the new Story individual within the ontology
    """
    story = onto2.Story(n1 + n2+"Story")
    for act in acts:
        #add the current act as property assertion to the story
        story.has_act.append(act)
    story.has_stop_trigger.append(stop_trigger)  #add the stop trigger to the story
    return story


def newEvent(actions, priority, startTrigger, n1, n2):
    """
    Creates an individual of class Event in the new ontology with name "n1n2Event". 

    Parameters:

    actions - an array with individuals of class Action. Use newSpeedAction, newTeleportActionWithPosition(), newTeleportActionWithRelativePositionAndOrientation(). newEnvironmentAction() to create those.

    priority - Can be overwrite, skip or parallel. Rules that govern interaction between events that belong to the same maneuver. 

    stop_trigger - an individual of the class StopTrigger. Use newStopTrigger() to create those. 

    n1, n2 - string used for the name of the individual. 

    Returns a reference to the new Story individual within the ontology
    """
    event = onto2.Event(n1 + n2 + "Event")
    event.has_priority.append(priority)
    event.has_start_trigger.append(startTrigger)
    for action in actions:
        event.has_action.append(action)
    return event

def newEgoVehicle(n1,n2,asset_name,bounding_box):
    """
    Creates an individual of class EgoVehicle in the new ontology with name "n1n2EgoVehicle". 

    Parameters:

    n1, n2 - string used for the name of the individual. 

    asset_name - a string with the name of the asset or a direct refence to the ontology individual.

    bounding_box - an individual of the class BoundingBox. Use newBoundingBox() to create those.

    Returns a reference to the new EgoVehicle individual within the ontology.
    """
    car = onto2.EgoVehicle(n1 + n2 + "EgoVehicle")
    asset = newAsset(asset_name)
    car.has_asset.append(asset)
    car.has_bounding_box.append(bounding_box)
    return car
    
def newCar(n1, n2, asset_name,bounding_box):
    """
    Creates an individual of class Car in the new ontology with name "n1n2Car". 

    Parameters:

    n1, n2 - string used for the name of the individual. 

    asset_name - a string with the name of the asset or a direct refence to the ontology individual.

    bounding_box - an individual of the class BoundingBox. Use newBoundingBox() to create those.

    Returns a reference to the new Car individual within the ontology.
    """
    car = onto2.Vehicle(n1 + n2 + "Car")
    asset = newAsset(asset_name)
    car.has_asset.append(asset)
    car.has_bounding_box.append(bounding_box)
    return car

def newSimulationCondition(simulation_time, delay, condition_edge, condition_rule,n1,n2):
    """
    Creates an individual of class SimulationTimeCondition in the new ontology with name "n1n2SimulationTimeCondition". 

    Parameters:
    
    simulation_time(double) - Time value of the simulation time condition. Unit: seconds

    condition_rule - an individual operator in the ontology, 
    
    possible values: lessThan, lessEqual , equalTo, notEqualTo, greaterEqual, greaterThan.

    n1, n2 - string used for the name of the individual. 

    Returns a reference to the new SimulationTimeCondition individual within the ontology.
    
    """
    condition = onto2.SimulationTimeCondition(n1 + n2 + "SimulationTimeCondition")
    condition.has_condition_edge.append(condition_edge)
    condition.has_condition_rule.append(condition_rule)
    condition.has_delay.append(delay)
    condition.has_simulation_time_condition.append(simulation_time)
    return condition

def newConditionGroup(conditions,n1,n2):
    """
    Creates an individual of class ConditionGroup in the new ontology with name "n1n2ConditionGroup". 

    Parameters:

    conditions - an array with individuals of class Condition. Use newSimulationTimeCondition(), newRelativeDistanceCondition(), newStoryboardStateCondition(), newTraveledDistanceCondition() new  to create those.

    n1, n2 - string used for the name of the individual 

    Returns a reference to the new ConditionGroup individual within the ontology
    """
    condition_group = onto2.ConditionGroup(n1+n2+"ConditionGroup")
    for condition in conditions:
        condition_group.has_condition.append(condition)
    return condition_group

def newAsset(string):
    """
    Creates a new asset in the ontology from the given string.

    Returns a reference to the new Asset individual within the ontology
    """
    return onto2.Asset(string)
   
def newStartTrigger(condition_groups, n1 , n2):
    """
    Creates an individual of class StartTrigger in the new ontology with name "n1n2StartTrigger". 

    Parameters:

    condition_groups - an array with individuals of class ConditionGroup. Use newConditionGroup() to create those.

    n1, n2 - string used for the name of the individual 

    Returns a reference to the new StartTrigger individual within the ontology
    """
    startTrigger = onto2.StartTrigger(n1+n2+"StartTrigger")
    for cg in condition_groups:
        startTrigger.has_condition_group.append(cg)
    return startTrigger

def newStopTrigger(condition_group, n1 , n2):
    """
    Creates an individual of class StopTrigger in the new ontology with name "n1n2StopTrigger". 

    Parameters:

    condition_groups - an array with individuals of class ConditionGroup. Use newConditionGroup() to create those.

    n1, n2 - string used for the name of the individual 

    Returns a reference to the new StopTrigger individual within the ontology
    """
    stopTrigger = onto2.StopTrigger(n1+n2+"StopTrigger")
    for cg in condition_group:
        stopTrigger.has_condition_group.append(cg)
    return stopTrigger    

def newPedestrian(n1,n2,asset_name,bounding_box):
    """
    Creates an individual of class Pedestrian in the new ontology with name "n1n2Pedestrian". 

    Parameters:

    n1, n2 - string used for the name of the individual. 

    asset_name - a string with the name of the asset or a direct refence to the ontology individual.

    bounding_box - an individual of the class BoundingBox. Use newBoundingBox() to create those.

    Returns a reference to the new Pedestrian individual within the ontology.
    """
    asset = newAsset(asset_name)
    pedestrian = onto2.Pedestrian(n1+n2+"Pedestrian")
    pedestrian.has_asset.append(asset)
    pedestrian.has_bounding_box.append(bounding_box)
    return pedestrian


def newTown(name):
    """
    Creates an individual of class Scenario in the new ontology with name "n1n2Scenario". 

    Parameters:

    name - 

    Returns a reference to the new Scenario individual within the ontology
    """
    return onto2.Town(name)


def newScenario(n1,n2,entities,storyboard,town):
    """
    Creates an individual of class Scenario in the new ontology with name "n1n2Scenario". 

    Parameters:

    entities - an array with individuals of classes with SubClassOf Entities. Use newCar(), newEgoVehicle(), newPedestrian(), newBicycle(), newMisc() to create those.

    storyboard - an individual of the class Storyboard. Use newStoryboard() to create those.

    town - an individual of the class Town. Use newTown() to create those. 

    n1, n2 - string used for the name of the individual 

    Returns a reference to the new Scenario individual within the ontology
    """
    scenario = onto2.Scenario(n1+n2+"Scenario")
    for entity in entities:
        scenario.has_entity.append(entity)
    scenario.has_storyboard.append(storyboard)
    scenario.has_town.append(town)
    return scenario
    
    
def newInit(n1,n2,actions):
    """
    Creates an individual of class Init in the new ontology with name "n1n2Init". 

    Parameters:

    n1, n2 - string used for the name of the individual 

    actions - an array with individual of class Action. use newSpeedAction(), newTeleportAction(), newEnvironmentAction() etc to create those.

    Returns a reference to the new Init individual within the ontology
    """
    return_init = onto2.Init(n1 + n2+"Init")
    for action in actions:
        return_init.has_init_action.append(action)
    return return_init
       
    
def newStoryboard(n1,n2, init, stories):
    """
    Creates an individual of class Storyboard in the new ontology with name "n1n2Storyboard".

    Parameters:

    n1, n2 - string used for the name of the individual
    
    init - an individual of the class Init. Use newInit() to create this.

    stories - an array with individual of class Story. Use newStory() to create those.
    
    Returns a reference to the new Storyboard individual within the ontology
    """
    storyboard = onto2.Storyboard(n1+n2+"Storyboard")
    storyboard.has_story.append(stories)
    storyboard.has_init.append(init)
    return storyboard
    
    
def newRelativeLaneChangeAction(n1,n2,lane_id,entity,offset,transition_dynamics):
    """
    Creates an individual of class Init in the new ontology with name "n1n2RelativeLaneChangeAction". 

    Parameters:

    n1, n2 - string used for the name of the individual 

    lane_id - ID of the current lane (ID of a lane in road network).

    entity - an entity reference.
    
    offset - Lane offset with respect to the reference entity's current lane position.

    transition_dynamics - an individual of the class TransitionDynamics. Use newTransitionDynamics to create those.

    Returns a reference to the new RelativeLaneChangeAction individual within the ontology
    """
    lane_action = onto2.RelativeLaneChangeAction(n1 + n2+"RelativeLaneChangeAction")
    lane_action.has_target_lane_id.append(lane_id)
    lane_action.has_entity_ref.append(entity)
    lane_action.has_offset.append(offset)
    lane_action.has_transition_dynamics.append(transition_dynamics)
    return lane_action


def newRelativeDistanceCondition(n1,n2, distance,condition_rule,distance_type,target_entity,delay,condition_edge):
    """
    Creates an individual of class RelativeDistanceCondition in the new ontology with name "n1n2RelativeDistanceCondition".

    Parameters:

    n1, n2 - string used for the name of the individual.

    distance(double) - the distance value.

    condition_rule - an individual operator in the ontology.
    
    possible values: lessThan, lessEqual , equalTo, notEqualTo, greaterEqual, greaterThan.

    distance_type - the domain the distance is calculated.

    possible values: longitudinal, lateral, cartesianDistance, euclideanDistance
    
    target_entity - target entity reference
    
    delay(int) - Time elapsed after the edge condition is verified, until the condition returns true to the scenario.
    
    condition_edge - Specifies the edge when the condition is evaluated to true.
    
    possible values: rising, falling, none.
    
    Returns a reference to the new RelativeLaneChangeAction individual within the ontology
    """
    relative_distance_condition = onto2.RelativeDistanceCondition(n1 + n2 + "RelativeDistanceCondtion")
    relative_distance_condition.has_distance_value.append(distance)
    relative_distance_condition.has_target_reference.append(target_entity)
    relative_distance_condition.has_condition_rule.append(condition_rule)
    relative_distance_condition.has_relative_distance_type.append(distance_type)
    relative_distance_condition.has_condition_edge.append(condition_edge)
    relative_distance_condition.has_delay.append(delay)
    return relative_distance_condition

def newStoryboardElementStateCondition(n1,n2,state,element_ref, delay, condition_edge):
    """
    Creates an individual of class StoryboardElementStateCondition in the new ontology with name "n1n2StoryboardElementStateCondition".

    Parameters:

    n1, n2 - string used for the name of the individual
    
    state - an individual of the class State. The states and the transitions that can be used to define a StoryboardElementStateCondition.
    
    possible values: startTransition, endTransition, stopTransition, skipTransition, completeState, runningState, standbyState
    
    element_ref - reference to the element, which has to reach the previously mentioned state. This should be an individual of the ontology.
    
    delay(int) - Time elapsed after the edge condition is verified, until the condition returns true to the scenario.
    
    condition_edge - Specifies the edge when the condition is evaluated to true.
    
    possible values: rising, falling, none.
    
    Returns a reference to the new StoryboardElementStateCondition individual within the ontology
    """
    storyboard_element_state_condition = onto2.StoryboardElementStateCondition(n1+n2+"StoryboardElementStateCondition")
    storyboard_element_state_condition.has_state.append(state)
    storyboard_element_state_condition.has_storyboard_element_ref.append(element_ref)
    storyboard_element_state_condition.has_delay.append(delay)
    storyboard_element_state_condition.has_condition_edge.append(condition_edge)
    return storyboard_element_state_condition


def newTimeOfDay(n1,n2,animation,year,month,day,hour,minute,second):
    """
    Creates an individual of class TimeOfDay in the new ontology with name "n1n2TimeOfDay".

    Parameters:

    n1, n2 - string used for the name of the individual
    
    animation - animation(boolean) - If true, the timeofday is animated with progressing simulation time, e.g. in order to animate the position of the sun.
    
    year - Specifies the year of the individual.
    
    month - Specifies the month of the individual.
    
    day - Specifies the day of the individual.
    
    hour - Specifies the hour of the individual.
    
    minute - Specifies the minutes of the individual.
    
    second - Specifies the seconds of the individual.
    
    Returns a reference to the new TimeOfDay individual within the ontology
    """
    time_of_day = onto2.TimeOfDay(n1 + n2 + "TimeOfDay")
    time_of_day.has_animation.append(animation)
    time_of_day.has_year.append(year)
    time_of_day.has_month.append(month)
    time_of_day.has_day.append(day)
    time_of_day.has_hour.append(hour)
    time_of_day.has_minute.append(minute)
    time_of_day.has_second.append(second)
    return time_of_day


def newBoundingBox(n1,n2,heigth, length,width,x,y,z):
    """
    Creates an individual of class BoundingBox in the new ontology with name "n1n2BoundingBox".

    Parameters:

    n1, n2 - string used for the name of the individual.
    
    The next 3 dimensions for a three dimensional box. Width, length and height are the absolute extensions in the (y,x,z) coordinate system of the entity's local coordinate system.
    
    height - Height of the entity's bounding box.
    
    length - Length of the entity's bounding box.
    
    width - Width of the entity's bounding box.
    
    x, y and z represent the geometrical center of the bounding box expressed in coordinates that refer to the coordinate system of the entity.
    
    x - Center offset in x direction.
    
    y - Center offset in y direction.
    
    z - Center offset in z direction.
    
    Returns a reference to the new BoundingBox individual within the ontology.
    """
    bounding_box = onto2.BoundingBox(n1 + n2 + "BoundingBox")
    bounding_box.has_heigth.append(heigth)
    bounding_box.has_length.append(length)
    bounding_box.has_width.append(width)
    bounding_box.has_x_center.append(x)
    bounding_box.has_y_center.append(y)
    bounding_box.has_z_center.append(z)
    return bounding_box


def newFog(n1,n2, visual_range, bounding_box):
    """
    Creates an individual of class Fog in the new ontology with name "n1n2Fog".

    Parameters:

    n1, n2 - string used for the name of the individual

    visual_range(int) - the visibility range of the fog. Over 100, there is no fog.
    
    bounding_box - an individual of the class BoundingBox. Use newBoundingBox() to create those.
    
    Returns a reference to the new Fog individual within the ontology
    """
    fog = onto2.Fog(n1 + n2 + "Fog")
    fog.has_bounding_box.append(bounding_box)
    fog.has_visual_range.append(visual_range)
    return fog


def newPrecipitation(n1,n2,intensity,precipitation_type):
    """
    Creates an individual of class Precipitation in the new ontology with name "n1n2Precipitation".

    Parameters:

    n1, n2 - string used for the name of the individual

    intensity - The intensity of the precipitation. Range [0..1]
    
    precipitation_type - Type of the precipitation
    
    possible values: dry, rain, snow

    Returns a reference to the new Precipitation individual within the ontology
    """
    precipitation = onto2.Precipitation(n1 + n2 + "Precipitation")
    precipitation.has_intensity.append(intensity)
    precipitation.has_precipitation_type.append(precipitation_type)
    return precipitation


def newSun(n1,n2,elevation,azimuth,intensity):
    """
    Creates an individual of class Sun in the new ontology with name "n1n2Sun".

    Parameters:

    n1, n2 - string used for the name of the individual

    intensity(double) - Illuminance of the sun, direct sunlight is around 100,00 lx. Unit: lux
    
    azimuth - Azimuth of the sun, counted counterclockwise, 0=north, PI/2 = east, PI=south, 3/2 PI=west. Unit: radian; Range: [0..2PI].
    
    elevation - Solar elevation angle, 0=x/y plane, PI/2=zenith. Unit: rad; Range: [-PI..PI]
    
    Returns a reference to the new Sun individual within the ontology
    """
    sun = onto2.Sun(n1 + n2 + "Sun")
    sun.has_elevation.append(elevation)
    sun.has_azimuth.append(azimuth)
    sun.has_intensity.append(intensity)
    return sun


def newWeather(n1,n2,cloud_state,sun,fog,precipitation):
    """
    Creates an individual of class Weather in the new ontology with name "n1n2Weather".

    Parameters:

    n1, n2 - string used for the name of the individual

    cloud_state - an individual in the ontology of the class CloudState.
    
    possible values: skyOff, free, cloudy, overcast, rainy
    
    sun - an individual of the class Sun. Use newSun() to create this.
    
    fog - an individual of the class Fog. Use newFog() to create this.
    
    precipitation - an indvidual of the class Precipitation. Use newPrecipitation() to create this.
    
    Returns a reference to the new Weather individual within the ontology
    """
    weather = onto2.Weather(n1+n2+"Weather")
    weather.has_fog.append(fog)
    weather.has_sun.append(sun)
    weather.has_precipitation.append(precipitation)
    weather.has_cloud_state.append(cloud_state)
    return weather

def newRoadCondition(n1,n2,friction_scale_factor):
    """
    Definition of the road friction scale factor.
    Creates an individual of class RoadCondition in the new ontology with name "n1n2RoadCondition".

    Parameters:

    n1, n2 - string used for the name of the individual

    friction_scale_factor(double) - friction scale factor.
    
    Returns a reference to the new RoadCondition individual within the ontology
    """
    road_condition = onto2.RoadCondition(n1+n2+"RoadCondition")
    road_condition.has_friction_scale_factor.append(friction_scale_factor)
    return road_condition


def newEnvironment(n1,n2,time_of_day,weather,road_condition):
    """
    Creates an individual of class Environment in the new ontology with name "n1n2Environment".

    Parameters:

    n1, n2 - string used for the name of the individual

    time_of_day - individual of the class TimeOfDay. Use newTimeOfDay() to create this.
    
    weather - individual of the class Weather. Use newWeather() to create this.
    
    road_condition - individual of the class RoadCondition. Use newRoadCondition() to create this.
    
    Returns a reference to the new Environment individual within the ontology
    """
    environment = onto2.Environment(n1+n2+"Environment")
    environment.has_time_of_day.append(time_of_day)
    environment.has_weather.append(weather)
    environment.has_road_condition.append(road_condition)
    return environment


def newEnvironmentAction(n1,n2,environment):
    """
    Creates an individual of class EnvironmentAction in the new ontology with name "n1n2EnvironmentAction".

    Parameters:

    n1, n2 - string used for the name of the individual
    
    environment - individual of the class Environment. Use newEnvironment() to create this.
    
    Returns a reference to the new EnvironmentAction individual within the ontology
    """
    environment_action = onto2.EnvironmentAction(n1 + n2 + "EnvironmentAction")
    environment_action.has_environment.append(environment)
    return environment_action


def newTransitionDynamics(n1,n2,dynamic_shape,dynamics_dimension,value):
    """
    Creates an individual of class TransitionDynamics in the new ontology with name "n1n2TransitionDynamics".

    Parameters:

    n1, n2 - string used for the name of the individual
    
    dynamic_shape - The shape of the transition function f(x) between current and target value.
    
    possible values: linear, step, cubic, sinusoidal
    
    dynamics_dimension - The semantics of the value
    
    possible values: rate, time, distance
    
    value - The value for a predefined rate.
    
    Returns a reference to the new TransitionDynamics individual within the ontology
    """
    transition_dynamics = onto2.TransitionDynamics(n1+n2+"TransitionDynamics")
    transition_dynamics.has_dynamics_shape.append(dynamic_shape)
    transition_dynamics.has_dynamics_dimension.append(dynamics_dimension)
    transition_dynamics.has_value.append(value)
    return transition_dynamics


def newTraveledDistanceCondition(n1,n2,entity_ref,distance,condition_edge):
    """
    Creates an individual of class TraveledDistanceCondition in the new ontology with name "n1n2TraveledDistanceCondition".

    Parameters:

    n1, n2 - string used for the name of the individual.
    
    entity_ref - a reference to the entity individual in the ontology.
    
    distance - traveled distance, to trigger the condition.
    
    condition_edge - Specifies the edge when the condition is evaluated to true.
    
    possible values: rising, falling, none.
    
    
    Returns a reference to the new TraveledDistanceCondition individual within the ontology
    """
    traveled_distance_condition = onto2.TraveledDistanceCondition(n1+n2+"TraveledDistanceCondition")
    traveled_distance_condition.has_entity_ref.append(entity_ref)
    traveled_distance_condition.has_traveled_distance_value.append(distance)
    traveled_distance_condition.has_condition_edge.append(condition_edge)
    return traveled_distance_condition


    ##Cut In ontology Scenario
def cutInOntology(n1,n2,filename):


    transition_dynamics = newTransitionDynamics(n1,n2+"15",onto2.step,onto2.time,0)
    #transition_dynamics1 = newTransitionDynamics(n1,n2+"16",onto2.step,onto2.distance,0)
    car_asset = getCarAssets()[1]
    bounding_box_ego = newBoundingBox(n1,n2+"cut_in",10,10,10,10,10,10)
    ego_init_speed = 16.6666
    cut_in_speed = 22.222222
    ##Scenario
    ego_vehicle = onto2.ego_vehicle
    cut_in_vehicle = newCar(n1,n2+"_cutIn",car_asset,bounding_box_ego)
    entities = []
    entities.append(ego_vehicle)
    entities.append(cut_in_vehicle)
    ego_speed_action_init = newSpeedAction(ego_vehicle,ego_init_speed,transition_dynamics,n1,n2+"Ego")
    cut_in_speed_action_init = newSpeedAction(cut_in_vehicle,cut_in_speed,transition_dynamics,n1,n2+"CutIn")
    init_actions = [ego_speed_action_init,cut_in_speed_action_init]
    ego_teleport_action = newTeleportActionWithPosition(ego_vehicle,5,50,0,48,n1,n2+"Ego")
    init_actions.append(ego_teleport_action)
    cut_teleport_action = newTeleportActionWithPosition(cut_in_vehicle,5,20,0,49,n1,n2+"CutIn")
    init_actions.append(cut_teleport_action)
    init_actions.append(onto2.def_env_action)
    init_scenario = newInit(n1, n2, init_actions)
    
    ##Story
    ##Actions
    lane_change_transition_dynamics = newTransitionDynamics(n1,n2+"Lane",onto2.cubic,onto2.distance,22)
    cut_in_lane_change_action_right = newRelativeLaneChangeAction(n1,n2+"CutIn",1,cut_in_vehicle,0,lane_change_transition_dynamics)
    cut_in_lane_change_action_left = newRelativeLaneChangeAction(n1,n2+"Left",-1,cut_in_vehicle,0,lane_change_transition_dynamics)
    #Start triggers
    zero_sim_condition = newSimulationCondition(0,0,onto2.rising, onto2.greaterThan,n1,n2+"zero")
    condtion_group_zero = newConditionGroup([zero_sim_condition],n1,n2+"Zero")
    start_trigger_zero = newStartTrigger([condtion_group_zero],n1,n2+"Zero")

    three_sim_condition = newSimulationCondition(3,0,onto2.rising, onto2.greaterThan ,n1,n2+"3")
    condtion_group_three = newConditionGroup([three_sim_condition],n1,n2+"Three")
    start_trigger_three = newStartTrigger([condtion_group_three],n1,n2+"Three")
    relative_distance_condition = newRelativeDistanceCondition(n1,n2,15,onto2.lessThan,onto2.cartesianDistance,cut_in_vehicle,0,onto2.rising)
    relative_distance_condition2 = newRelativeDistanceCondition(n1,n2+"Start",17,onto2.lessThan,onto2.cartesianDistance,cut_in_vehicle,0,onto2.rising)
    condition_group_distance = newConditionGroup([relative_distance_condition],n1,n2+"Distance")
    condition_group_distance2 = newConditionGroup([relative_distance_condition2],n1,n2+"Distance2")
    start_trigger_distance = newStartTrigger([condition_group_distance],n1,n2+"DistanceTrigger")
    start_trigger_distance2 = newStartTrigger([condition_group_distance2],n1,n2+"DistanceTrigger2")
    
    #Event
    event_cut_in_right = newEvent([cut_in_lane_change_action_right], onto2.overwrite,start_trigger_distance,n1,n2+"CutIn")
    #StoryboardElementCondition Start Trigger
    storyboard_element_state_condition_event1 = newStoryboardElementStateCondition(n1,n2+"cut_in_event",onto2.completeState,event_cut_in_right,0 ,onto2.none)
    traveled_distance_condition = newTraveledDistanceCondition(n1,n2+"70",cut_in_vehicle,70,onto2.none)
    conditionList = []
    conditionList.append(storyboard_element_state_condition_event1)
    conditionList.append(traveled_distance_condition)
    condtion_group_complete_state_and_distance = newConditionGroup(conditionList,n1,n2 + "completeState")
    start_trigger_state = newStartTrigger([condtion_group_complete_state_and_distance],n1,n2+"StateTrigger")

    #Event2
    event_cut_in_left = newEvent([cut_in_lane_change_action_left],onto2.overwrite,start_trigger_state,n1,n2+"CutInLeft")

    #Longersimulation
    keep_longer_action = newSpeedAction(ego_vehicle,ego_init_speed,transition_dynamics,n1,n2+"longer")
    storyboard_element_state_condition_longer = newStoryboardElementStateCondition(n1,n2+"KeepLonger",onto2.completeState,event_cut_in_left, 5,onto2.none)
    condtion_group_complete_state_longer = newConditionGroup([storyboard_element_state_condition_longer],n1,n2 + "completeState1")
    start_trigger_state_longer = newStartTrigger([condtion_group_complete_state_longer],n1,n2+"StateTrigger1")
    event_keep_longer = newEvent([keep_longer_action],onto2.overwrite,start_trigger_state_longer,n1,n2+"KeepLonger")

    maneuver = newManeuver([event_cut_in_right,event_cut_in_left,event_keep_longer],n1,n2)
    maneuver_group = newManeuverGroup([maneuver],cut_in_vehicle,n1,n2)
    
    ##stop triggers
    sim_condition_30 = newSimulationCondition(30,0,onto2.rising,onto2.greaterThan,n1,n2+"30")
    condtion_group_30 = newConditionGroup([sim_condition_30],n1,n2+"30")
    stop_trigger_30 = newStopTrigger([condtion_group_30],n1,n2+"30")
    
    
    scenario_act = newAct([maneuver_group],start_trigger_distance2,stop_trigger_30,n1,n2)
    story = newStory([scenario_act],stop_trigger_30,n1,n2)
    storyboard = newStoryboard(n1,n2,init_scenario,story)
    scenario = newScenario(n1,n2,entities, storyboard,onto2.Town04)
    #ego_
    print("done cut_in")
    onto2.save(filename)
    return scenario

def fallingSigns(n1,n2,filename):
    ego_vehicle = onto2.ego_vehicle
    ego_init_speed = 7
    transition_dynamics = newTransitionDynamics(n1,n2,onto2.step,onto2.distance,15)
    ego_speed_action_init = newSpeedAction(ego_vehicle,ego_init_speed,transition_dynamics,n1,n2+"Ego")
    ego_teleport_action = newTeleportActionWithPosition(ego_vehicle,-1,0,2,1,n1,n2+"Ego")
    streetsign1 = newMisc(n1,n2+"1","static.prop.streetsign01")
    streetsign2 = newMisc(n1,n2+"2","static.prop.streetsign01")
    streetsign3 = newMisc(n1,n2+"3","static.prop.streetsign01")
    ss1_teleport_action = newTeleportActionWithPosition(streetsign1,1,75,0,1,n1,n2+"ss1_init")
    ss2_teleport_action = newTeleportActionWithPosition(streetsign2,-2,70,0,1,n1,n2+"ss2_init")
    ss3_teleport_action = newTeleportActionWithPosition(streetsign3,1,65,0,1,n1,n2+"ss3_init")
    environment_action = onto2.def_env_action
    init_scenario = newInit(n1,n2, [ego_speed_action_init,ego_teleport_action,ss1_teleport_action,environment_action,ss2_teleport_action,ss3_teleport_action])

    #StartTrigger
    zero_sim_condition = newSimulationCondition(0,0,onto2.rising, onto2.greaterThan,n1,n2+"zero")
    condtion_group_zero = newConditionGroup([zero_sim_condition],n1,n2+"Zero")
    start_trigger_zero = newStartTrigger([condtion_group_zero],n1,n2+"Zero")

    #DistanceTrigger
    traveled_distance_condition = newTraveledDistanceCondition(n1,n2+"70",ego_vehicle,50,onto2.none)
    condtion_group_distance = newConditionGroup([traveled_distance_condition],n1,n2+"Distance")
    start_trigger_distance = newStartTrigger([condtion_group_distance],n1,n2+"Distance")
     
    #StopTrigger
    sim_condition_30 = newSimulationCondition(30,0,onto2.rising,onto2.greaterThan,n1,n2+"30")
    condtion_group_30 = newConditionGroup([sim_condition_30],n1,n2+"30")
    stop_trigger_30 = newStopTrigger([condtion_group_30],n1,n2+"30")

    #bbq teleport action
    bbq_teleport_action_story = newTeleportActionWithRelativePositionAndOrientation(n1,n2+"ss1",streetsign1,0,0,0,0.0,-0.5,0)
    event1 = newEvent([bbq_teleport_action_story],onto2.overwrite,start_trigger_distance,n1,n2+"ss1_distance")
    maneuver1 = newManeuver([event1],n1,n2+"ss1")
    mg1 = newManeuverGroup([maneuver1],streetsign1,n1,n2+"ss1")
    #ss2
    bbq_teleport_action_story2 = newTeleportActionWithRelativePositionAndOrientation(n1,n2+"ss2",streetsign2,0,0,0,0.0,-0.5,0)
    event2 = newEvent([bbq_teleport_action_story2],onto2.overwrite,start_trigger_distance,n1,n2+"ss2_distance")
    maneuver2 = newManeuver([event2],n1,n2+"ss2")
    mg2 = newManeuverGroup([maneuver2],streetsign2,n1,n2+"ss2")
    ##ss3
    bbq_teleport_action_story3 = newTeleportActionWithRelativePositionAndOrientation(n1,n2+"ss3",streetsign3,0,0,0,0.0,-0.5,0)
    event3 = newEvent([bbq_teleport_action_story3],onto2.overwrite,start_trigger_distance,n1,n2+"ss3_distance")
    maneuver3 = newManeuver([event3],n1,n2+"ss3")
    mg3 = newManeuverGroup([maneuver3],streetsign3,n1,n2+"ss3")

    #ego speed
    keep_longer_action = newSpeedAction(ego_vehicle,5,transition_dynamics,n1,n2+"longer")
    storyboard_element_state_condition_event1 = newStoryboardElementStateCondition(n1,n2+"speedchange",onto2.completeState, event1, 10,onto2.none)
    condtion_group_complete_state = newConditionGroup([storyboard_element_state_condition_event1],n1,n2 + "completeState")
    start_trigger_state = newStartTrigger([condtion_group_complete_state],n1,n2+"StateTrigger")
    event_keep_longer = newEvent([keep_longer_action],onto2.overwrite,start_trigger_state,n1,n2+"KeepLonger")
    maneuver_ego = newManeuver([event_keep_longer],n1,n2)
    maneuver_group = newManeuverGroup([maneuver_ego],ego_vehicle,n1,n2)

    #Act and rest of the scenario
    scenario_act = newAct([mg1,maneuver_group,mg2,mg3],start_trigger_zero,stop_trigger_30,n1,n2+"1")
    story = newStory([scenario_act],stop_trigger_30,n1,n2)
    storyboard = newStoryboard(n1,n2,init_scenario,story)
    scenario = newScenario(n1,n2,[ego_vehicle,streetsign1,streetsign2,streetsign3], storyboard, onto2.Town04)
    onto2.save(filename)
    return scenario
    ##Fog Ontology scenario

def intoFogOntology(n1,n2,filename):
    ego_init_speed = 15
    bounding_box_ego = newBoundingBox(n1,n2+"Ego",10,10,10,10,10,10)
    car_assets = getCarAssets()
    
    ego_vehicle = onto2.ego_vehicle
    transition_dynamics = newTransitionDynamics(n1,n2,onto2.step,onto2.distance,15)
    ego_teleport_action = newTeleportActionWithPosition(ego_vehicle,5,50,0,48,n1,n2+"Ego")
    ego_speed_action_init = newSpeedAction(ego_vehicle,ego_init_speed,transition_dynamics,n1,n2+"Ego")
    init_weather = onto2.def_env_action
    init_scenario = newInit(n1, n2, [ego_speed_action_init,ego_teleport_action,init_weather])
    #Environment Action
    time_of_day = newTimeOfDay(n1,n2+"1200","true",2021,1,7,13,10,10)
    road_condition = newRoadCondition(n1,n2+"1",1)
    sun = newSun(n1,n2,1.3,0,0.8)
    bounding_box = newBoundingBox(n1,n2+"Fog_bb",10,10,10,10,10,10)
    fog = newFog(n1,n2,50,bounding_box)
    precipitation = newPrecipitation(n1,n2+"dry",1,onto2.dry)
    weather = newWeather(n1,n2+"foggy",onto2.free,sun,fog,precipitation)
    environment = newEnvironment(n1,n2,time_of_day,weather,road_condition)
    environment_action = newEnvironmentAction(n1,n2+"1",environment)

    #Triggers
    zero_sim_condition = newSimulationCondition(0,0,onto2.rising, onto2.greaterThan,n1,n2+"zero")
    condtion_group_zero = newConditionGroup([zero_sim_condition],n1,n2+"Zero")
    start_trigger_zero = newStartTrigger([condtion_group_zero],n1,n2+"Zero")

    traveled_distance_condition = newTraveledDistanceCondition(n1,n2+"70",ego_vehicle,70,onto2.none)
    condtion_group_distance = newConditionGroup([traveled_distance_condition],n1,n2+"Distance")
    start_trigger_distance = newStartTrigger([condtion_group_distance],n1,n2+"Distance")

    event_weather_change = newEvent([environment_action], onto2.overwrite,start_trigger_distance,n1,n2+"weather_change")
    ##Keep the scenario going on for a bit longer
    keep_longer_action = newSpeedAction(ego_vehicle,5,transition_dynamics,n1,n2+"longer")
    storyboard_element_state_condition_event1 = newStoryboardElementStateCondition(n1,n2+"weatherchange",onto2.completeState,event_weather_change, 5,onto2.none)
    condtion_group_complete_state = newConditionGroup([storyboard_element_state_condition_event1],n1,n2 + "completeState")
    start_trigger_state = newStartTrigger([condtion_group_complete_state],n1,n2+"StateTrigger")
    event_keep_longer = newEvent([keep_longer_action],onto2.overwrite,start_trigger_state,n1,n2+"KeepLonger")
    ##
    maneuver = newManeuver([event_weather_change,event_keep_longer],n1,n2)
    maneuver_group = newManeuverGroup([maneuver],ego_vehicle,n1,n2)

    ##stop triggers
    sim_condition_30 = newSimulationCondition(30,0,onto2.rising,onto2.greaterThan,n1,n2+"30")
    condtion_group_30 = newConditionGroup([sim_condition_30],n1,n2+"30")
    stop_trigger_30 = newStopTrigger([condtion_group_30],n1,n2+"30")
    

    scenario_act = newAct([maneuver_group],start_trigger_zero,stop_trigger_30,n1,n2)
    story = newStory([scenario_act],stop_trigger_30,n1,n2)
    storyboard = newStoryboard(n1,n2,init_scenario,story)
    scenario = newScenario(n1,n2,[ego_vehicle], storyboard, onto2.Town04)
    
    print("done environment")
    onto2.save(filename)
    return scenario
    
def ManyPedestriansOntology(n1,n2,filename):
    transition_dynamics = newTransitionDynamics(n1,n2,onto2.step,onto2.distance,15)
    scenarioTown = onto2.Town01
    amountPedestrians = 10
    newScenario = onto2.Scenario(n1 + n2 + "Scenario")
    newStoryboard = onto2.Storyboard(n1 + n2 + "Storyboard")
    newScenario.has_storyboard.append(newStoryboard)
    newScenario.has_town.append(scenarioTown)
    bounding_box = newBoundingBox(n1,n2+"bb",10,10,10,10,10,10)
    # entities
    pedestrian_assets = getPedestrianAssets()
    list_of_pedestrians = []
    # creating amountPedestrians pedestrians
    for i in range(amountPedestrians):
        ped = newPedestrian(n1,str(i),pedestrian_assets[i],bounding_box)
        list_of_pedestrians.append(ped)
    for i in range(amountPedestrians):
        ped = newPedestrian(n1,str(i),pedestrian_assets[i],bounding_box)
        newScenario.has_entity.append(ped)
    # story
    story = onto2.Story(n1 + n2 + "Story")
    newStoryboard.has_story.append(story)
    
    ego_vehicle = onto2.ego_vehicle
    newScenario.has_entity.append(ego_vehicle)
    # init
    new_init = onto2.Init(n1 + n2 + "Init1")
    #Ego Vehicle Actions
    actionTeleportEgo = newTeleportActionWithPosition(ego_vehicle, -1, 0, 0, 1, n1, n2+"EgoTeleport")
    actionSpeedEgo = newSpeedAction(ego_vehicle, 3,transition_dynamics, n1, n2 + "EgoSpeed")
    new_init.has_init_action.append(actionSpeedEgo)
    new_init.has_init_action.append(actionTeleportEgo)
    weatheraction = onto2.def_env_action
    new_init.has_init_action.append(weatheraction)
    newStoryboard.has_init.append(new_init)
    #act = onto2.Act(n1 + n2 + "Act")
    maneuver_groups = []
    # Add amountPedestrians pedestrians in the ontology
    s_jump = 20
    for i in range(amountPedestrians):  
        actions = []
        events = []
        place = 5
        pedestrian_assets = getPedestrianAssets()
        pedestrian = newPedestrian(n1,str(i),pedestrian_assets[i],bounding_box)
        if i%2 == 0:
            offset = -1
        else:
            offset = 0
        actionTeleport = newTeleportActionWithPosition(pedestrian, -1, s_jump, offset, 1, n1, str(i))
        actionSpeed = newSpeedAction(pedestrian, 2,transition_dynamics, n1, str(i))
        new_init.has_init_action.append(actionTeleport)
        new_init.has_init_action.append(actionSpeed)
        action3 = newSpeedAction(pedestrian, 8,transition_dynamics, n1 + "_0_" , str(i))
       # action4 = newTeleportActionWithPosition(pedestrian, 2, 22, 0, i+1, n1 + n3, str(i))
        actions.append(action3)
       # actions.append(action4)
        zero_sim_condition = newSimulationCondition(5,0,onto2.rising, onto2.greaterThan,n1,n2+"zero")
        condtion_group_zero = newConditionGroup([zero_sim_condition],n1,n2+"Zero")
        start_trigger_zero = newStartTrigger([condtion_group_zero],n1,n2+"Zero")
        event1 = newEvent(actions, onto2.overwrite, start_trigger_zero,n1, str(i))
        events.append(event1)
        maneuver1 = newManeuver(events, n1, str(i))
        maneuverGroup1 = newManeuverGroup([maneuver1], pedestrian, n1, str(i))
        maneuver_groups.append(maneuverGroup1)
        s_jump += 4
    sim_condition_30 = newSimulationCondition(30,0,onto2.rising,onto2.greaterThan,n1,n2+"30")
    condtion_group_30 = newConditionGroup([sim_condition_30],n1,n2+"30")
    stop_trigger_30 = newStopTrigger([condtion_group_30],n1,n2+"30")
    zero_sim_condition = newSimulationCondition(5,0,onto2.rising, onto2.greaterThan,n1,n2+"zero")
    condtion_group_zero = newConditionGroup([zero_sim_condition],n1,n2+"Zero")
    start_trigger_zero = newStartTrigger([condtion_group_zero],n1,n2+"Zero")

    act = newAct(maneuver_groups,start_trigger_zero,stop_trigger_30,n1,n2)    
    ##TODO add start triggers here to each event , creeate them all new
    story.has_act.append(act)
    # with ontoUAI:
    #    sync_reasoner_pellet(infer_property_values = True, infer_data_property_values = True)
    # print("done pedestrians")
    onto2.save(filename)

def BicycleOnOneWheelOntology(n1,n2,filename):
    ego_init_speed = 8
    bicycle_speed = 8
    bounding_box_bike = newBoundingBox(n1,n2+"Ego",10,10,10,10,10,10)
    ##Scenario
    transition_dynamics = newTransitionDynamics(n1,n2,onto2.step,onto2.distance,15)
    ego_vehicle = onto2.ego_vehicle
    bicycle = newBicycle(n1,n2+"bike",getBicycleAssets()[0],bounding_box_bike)
    entities = []
    entities.append(ego_vehicle)
    entities.append(bicycle)
    #Init Actions and init
    ego_speed_action_init = newSpeedAction(ego_vehicle,ego_init_speed,transition_dynamics,n1,n2+"Ego")
    bike_speed_action_init = newSpeedAction(bicycle,bicycle_speed,transition_dynamics,n1,n2+"bicycle")
    init_actions = [ego_speed_action_init,bike_speed_action_init]
    ego_teleport_action = newTeleportActionWithPosition(ego_vehicle,-1,0,0,1,n1,n2+"Ego")
    init_actions.append(ego_teleport_action)
    bike_teleport_action = newTeleportActionWithPosition(bicycle,1,80,0,1,n1,n2+"bicycle")
    init_actions.append(bike_teleport_action)
    init_actions.append(onto2.def_env_action)
    init_scenario = newInit(n1, n2, init_actions)
    #Triggers 
    zero_sim_condition = newSimulationCondition(0,0,onto2.rising, onto2.greaterThan,n1,n2+"zero")
    condtion_group_zero = newConditionGroup([zero_sim_condition],n1,n2+"Zero")
    start_trigger_zero = newStartTrigger([condtion_group_zero],n1,n2+"Zero")

    traveled_distance_condition = newTraveledDistanceCondition(n1,n2+"10",bicycle,20,onto2.rising)
    traveled_condition_group = newConditionGroup([traveled_distance_condition],n1,n2+"travel")
    start_trigger_distance = newStartTrigger([traveled_condition_group],n1,n2+"travel")
    
    #StopTrigger
    sim_condition_30 = newSimulationCondition(30,0,onto2.rising,onto2.greaterThan,n1,n2+"30")
    condtion_group_30 = newConditionGroup([sim_condition_30],n1,n2+"30")
    stop_trigger_30 = newStopTrigger([condtion_group_30],n1,n2+"30")

    #Event
    bicycle_teleport_action_story = newTeleportActionWithRelativePositionAndOrientation(n1,n2+"ss1",bicycle,0,0,0.7,0.7,0,0.7)
    event1 = newEvent([bicycle_teleport_action_story],onto2.overwrite,start_trigger_distance,n1,n2+"bbq_distance")
    maneuver1 = newManeuver([event1],n1,n2+"ss1")
    mg1 = newManeuverGroup([maneuver1],bicycle,n1,n2+"ss1")
    
    #keeplonger
    keep_longer_action = newSpeedAction(ego_vehicle,5,transition_dynamics,n1,n2+"longer")
    storyboard_element_state_condition_event1 = newStoryboardElementStateCondition(n1,n2+"speedchange",onto2.completeState, event1,1,onto2.none)
    condtion_group_complete_state = newConditionGroup([storyboard_element_state_condition_event1],n1,n2 + "completeState")
    start_trigger_state = newStartTrigger([condtion_group_complete_state],n1,n2+"StateTrigger")
    event_keep_longer = newEvent([keep_longer_action],onto2.overwrite,start_trigger_state,n1,n2+"KeepLonger")
    maneuver_ego = newManeuver([event_keep_longer],n1,n2)
    maneuver_group = newManeuverGroup([maneuver_ego],ego_vehicle,n1,n2)
    
    #rest
    scenario_act = newAct([mg1,maneuver_group],start_trigger_zero,stop_trigger_30,n1,n2+"1")
    story = newStory([scenario_act],stop_trigger_30,n1,n2) 
    storyboard = newStoryboard(n1,n2,init_scenario,story)
    scenario = newScenario(n1,n2,[ego_vehicle,bicycle], storyboard, onto2.Town01)
    onto2.save(filename)
    return scenario

def randomObjectOntology(n1,n2,filename):
    ego_vehicle = onto2.ego_vehicle
    ego_init_speed = 10
    transition_dynamics = newTransitionDynamics(n1,n2,onto2.step,onto2.distance,15)
    ego_speed_action_init = newSpeedAction(ego_vehicle,ego_init_speed,transition_dynamics,n1,n2+"Ego")
    ego_teleport_action = newTeleportActionWithPosition(ego_vehicle,-1,0,2,1,n1,n2+"Ego")
    barbeque1 = newMisc(n1,n2,"static.prop.vendingmachine")
    barbeque_teleport_action = newTeleportActionWithPosition(barbeque1,1,75,0,1,n1,n2+"bbq_init")
    environment_action = onto2.def_env_action
    init_scenario = newInit(n1,n2, [ego_speed_action_init,ego_teleport_action,barbeque_teleport_action,environment_action])

    #StartTrigger
    zero_sim_condition = newSimulationCondition(0,0,onto2.rising, onto2.greaterThan,n1,n2+"zero")
    condtion_group_zero = newConditionGroup([zero_sim_condition],n1,n2+"Zero")
    start_trigger_zero = newStartTrigger([condtion_group_zero],n1,n2+"Zero")

    #DistanceTrigger
    traveled_distance_condition = newTraveledDistanceCondition(n1,n2+"70",ego_vehicle,50,onto2.none)
    condtion_group_distance = newConditionGroup([traveled_distance_condition],n1,n2+"Distance")
    start_trigger_distance = newStartTrigger([condtion_group_distance],n1,n2+"Distance")
     
    #StopTrigger
    sim_condition_30 = newSimulationCondition(30,0,onto2.rising,onto2.greaterThan,n1,n2+"30")
    condtion_group_30 = newConditionGroup([sim_condition_30],n1,n2+"30")
    stop_trigger_30 = newStopTrigger([condtion_group_30],n1,n2+"30")

    #bbq teleport action
    bbq_teleport_action_story = newTeleportActionWithRelativePositionAndOrientation(n1,n2,barbeque1,0,0,0,0.0,-0.5,0)
    event1 = newEvent([bbq_teleport_action_story],onto2.overwrite,start_trigger_distance,n1,n2+"bbq_distance")
    maneuver1 = newManeuver([event1],n1,n2+"bbq")
    mg1 = newManeuverGroup([maneuver1],barbeque1,n1,n2+"bbq")

    #ego speed
    keep_longer_action = newSpeedAction(ego_vehicle,5,transition_dynamics,n1,n2+"longer")
    storyboard_element_state_condition_event1 = newStoryboardElementStateCondition(n1,n2+"speedchange",onto2.completeState, event1, 20,onto2.none)
    condtion_group_complete_state = newConditionGroup([storyboard_element_state_condition_event1],n1,n2 + "completeState")
    start_trigger_state = newStartTrigger([condtion_group_complete_state],n1,n2+"StateTrigger")
    event_keep_longer = newEvent([keep_longer_action],onto2.overwrite,start_trigger_state,n1,n2+"KeepLonger")
    maneuver_ego = newManeuver([event_keep_longer],n1,n2)
    maneuver_group = newManeuverGroup([maneuver_ego],ego_vehicle,n1,n2)

    #Act and rest of the scenario
    scenario_act = newAct([mg1,maneuver_group],start_trigger_zero,stop_trigger_30,n1,n2+"1")
    story = newStory([scenario_act],stop_trigger_30,n1,n2)
    storyboard = newStoryboard(n1,n2,init_scenario,story)
    scenario = newScenario(n1,n2,[ego_vehicle,barbeque1], storyboard, onto2.Town04)
    onto2.save(filename)
    return scenario


def runningIntoCar(n1,n2,filename):

    bounding_box_object = newBoundingBox(n1,n2+"Ego",10,10,10,10,10,10)
    ##Scenario
    transition_dynamics = newTransitionDynamics(n1,n2,onto2.step,onto2.distance,15)
    ego_vehicle = onto2.ego_vehicle
    pedestrian = newPedestrian(n1,n2+"pedestrian",getPedestrianAssets()[0],bounding_box_object)
    ego_teleport_action = newTeleportActionWithPosition(ego_vehicle,-1,0,0,1,n1,n2+"Ego")
    ped_teleport_action = newTeleportActionWithPosition(pedestrian,2,40,2,1,n1,n2+"pedestrian")
    ego_speed_action_init = newSpeedAction(ego_vehicle,5,transition_dynamics,n1,n2+"Ego")
    weather_action = onto2.def_env_action
    init_scenario = newInit(n1, n2, [ego_teleport_action,ped_teleport_action,ego_speed_action_init,weather_action])

    #triggers
    zero_sim_condition = newSimulationCondition(0,0,onto2.rising, onto2.greaterThan,n1,n2+"zero")
    condtion_group_zero = newConditionGroup([zero_sim_condition],n1,n2+"Zero")
    start_trigger_zero = newStartTrigger([condtion_group_zero],n1,n2+"Zero")

    relative_distance_condition = newRelativeDistanceCondition(n1,n2,17,onto2.lessThan,onto2.cartesianDistance,ego_vehicle,0,onto2.rising)
    condition_group_distance = newConditionGroup([relative_distance_condition],n1,n2+"Distance")
    start_trigger_distance = newStartTrigger([condition_group_distance],n1,n2+"DistanceTrigger")

    #StopTrigger
    sim_condition_30 = newSimulationCondition(30,0,onto2.rising,onto2.greaterThan,n1,n2+"30")
    condtion_group_30 = newConditionGroup([sim_condition_30],n1,n2+"30")
    stop_trigger_30 = newStopTrigger([condtion_group_30],n1,n2+"30")

    ## Start running event
    pedestrian_teleport_action_story = newTeleportActionWithRelativePositionAndOrientation(n1,n2+"ss1",pedestrian,0,0,0,0,0,1.5)
    pedestrian_speed_action = newSpeedAction(pedestrian,7,transition_dynamics,n1,n2+"pedestrian")
    event1 = newEvent([pedestrian_teleport_action_story,pedestrian_speed_action],onto2.overwrite,start_trigger_distance,n1,n2+"bbq_distance")
    maneuver1 = newManeuver([event1],n1,n2+"ss1")
    mg1 = newManeuverGroup([maneuver1],pedestrian,n1,n2+"ss1")

    #keeplonger
    keep_longer_action = newSpeedAction(ego_vehicle,5,transition_dynamics,n1,n2+"longer")
    storyboard_element_state_condition_event1 = newStoryboardElementStateCondition(n1,n2+"speedchange",onto2.completeState, event1,1,onto2.none)
    condtion_group_complete_state = newConditionGroup([storyboard_element_state_condition_event1],n1,n2 + "completeState")
    start_trigger_state = newStartTrigger([condtion_group_complete_state],n1,n2+"StateTrigger")
    event_keep_longer = newEvent([keep_longer_action],onto2.overwrite,start_trigger_state,n1,n2+"KeepLonger")
    maneuver_ego = newManeuver([event_keep_longer],n1,n2)
    maneuver_group = newManeuverGroup([maneuver_ego],ego_vehicle,n1,n2)

    #rest
    scenario_act = newAct([mg1,maneuver_group],start_trigger_zero,stop_trigger_30,n1,n2+"1")
    story = newStory([scenario_act],stop_trigger_30,n1,n2)
    storyboard = newStoryboard(n1,n2,init_scenario,story)
    scenario = newScenario(n1,n2,[ego_vehicle,pedestrian], storyboard, onto2.Town01)
    print("the fuck why doesnt it save")
    onto2.save(filename)
    return scenario

def testMerge(n1,n2,filename):
    ego_init_speed = 8
    bicycle_speed = 8
    bounding_box_bike = newBoundingBox(n1,n2+"Ego",10,10,10,10,10,10)
    ##Scenario
    transition_dynamics = newTransitionDynamics(n1,n2,onto2.step,onto2.distance,15)
    ego_vehicle = onto2.ego_vehicle
    bicycle = newBicycle(n1,n2+"bike",getBicycleAssets()[0],bounding_box_bike)
    entities = []
    entities.append(ego_vehicle)
    entities.append(bicycle)
    #Init Actions and init
    ego_speed_action_init = newSpeedAction(ego_vehicle,ego_init_speed,transition_dynamics,n1,n2+"Ego")
    bike_speed_action_init = newSpeedAction(bicycle,bicycle_speed,transition_dynamics,n1,n2+"bicycle")
    weather_change = onto2.def_env_action
    init_actions = [ego_speed_action_init,bike_speed_action_init,weather_change]
    ego_teleport_action = newTeleportActionWithPosition(ego_vehicle,-1,0,0,1,n1,n2+"Ego")
    init_actions.append(ego_teleport_action)
    bike_teleport_action = newTeleportActionWithPosition(bicycle,1,80,0,1,n1,n2+"bicycle")
    init_actions.append(bike_teleport_action)
    #Triggers 
    zero_sim_condition = newSimulationCondition(0,0,onto2.rising, onto2.greaterThan,n1,n2+"zero")
    condtion_group_zero = newConditionGroup([zero_sim_condition],n1,n2+"Zero")
    start_trigger_zero = newStartTrigger([condtion_group_zero],n1,n2+"Zero")

    traveled_distance_condition = newTraveledDistanceCondition(n1,n2+"10",bicycle,20,onto2.rising)
    traveled_condition_group = newConditionGroup([traveled_distance_condition],n1,n2+"travel")
    start_trigger_distance = newStartTrigger([traveled_condition_group],n1,n2+"travel")
    
    #StopTrigger
    sim_condition_30 = newSimulationCondition(30,0,onto2.rising,onto2.greaterThan,n1,n2+"30")
    condtion_group_30 = newConditionGroup([sim_condition_30],n1,n2+"30")
    stop_trigger_30 = newStopTrigger([condtion_group_30],n1,n2+"30")

    #Event
    bicycle_teleport_action_story = newTeleportActionWithRelativePositionAndOrientation(n1,n2+"ss1",bicycle,0,0,0.7,0.7,0,0.7)
    event1 = newEvent([bicycle_teleport_action_story],onto2.overwrite,start_trigger_distance,n1,n2+"bbq_distance")
    maneuver1 = newManeuver([event1],n1,n2+"ss1")
    mg1 = newManeuverGroup([maneuver1],bicycle,n1,n2+"ss1")
    
    #keeplonger
    keep_longer_action = newSpeedAction(ego_vehicle,5,transition_dynamics,n1,n2+"longer")
    storyboard_element_state_condition_event1 = newStoryboardElementStateCondition(n1,n2+"speedchange",onto2.completeState, event1,1,onto2.none)
    condtion_group_complete_state = newConditionGroup([storyboard_element_state_condition_event1],n1,n2 + "completeState")
    start_trigger_state = newStartTrigger([condtion_group_complete_state],n1,n2+"StateTrigger")
    event_keep_longer = newEvent([keep_longer_action],onto2.overwrite,start_trigger_state,n1,n2+"KeepLonger")
    maneuver_ego = newManeuver([event_keep_longer],n1,n2)
    maneuver_group = newManeuverGroup([maneuver_ego],ego_vehicle,n1,n2)
    maneuver_groups = []
    s_jump = 20
    amountPedestrians = 10
    list_of_pedestrians = []
    for i in range(amountPedestrians):  
        actions = []
        events = []
        place = 5
        pedestrian_assets = getPedestrianAssets()
        pedestrian = newPedestrian(n1,str(i),pedestrian_assets[i],bounding_box_bike)
        list_of_pedestrians.append(pedestrian)
        if i%2 == 0:
            offset = -1
        else:
            offset = 0
        actionTeleport = newTeleportActionWithPosition(pedestrian, -1, s_jump, offset, 1, n1, str(i))
        actionSpeed = newSpeedAction(pedestrian, 2,transition_dynamics, n1, str(i))
        init_actions.append(actionTeleport)
        init_actions.append(actionSpeed)
        action3 = newSpeedAction(pedestrian, 8,transition_dynamics, n1 + "_0_" , str(i))
        # action4 = newTeleportActionWithPosition(pedestrian, 2, 22, 0, i+1, n1 + n3, str(i))
        actions.append(action3)
        # actions.append(action4)
        zero_sim_condition = newSimulationCondition(5,0,onto2.rising, onto2.greaterThan,n1,n2+"zero")
        condtion_group_zero = newConditionGroup([zero_sim_condition],n1,n2+"Zero")
        start_trigger_zero = newStartTrigger([condtion_group_zero],n1,n2+"Zero")
        event1 = newEvent(actions, onto2.overwrite, start_trigger_zero,n1, str(i))
        events.append(event1)
        maneuver1 = newManeuver(events, n1, str(i))
        maneuverGroup1 = newManeuverGroup([maneuver1], pedestrian, n1, str(i))
        maneuver_groups.append(maneuverGroup1)
        s_jump += 4
    #rest
    init_scenario = newInit(n1, n2, init_actions)
    maneuver_groups.append(mg1)
    maneuver_groups.append(maneuver_group)
    scenario_act = newAct(maneuver_groups,start_trigger_zero,stop_trigger_30,n1,n2+"1")
    story = newStory([scenario_act],stop_trigger_30,n1,n2) 
    storyboard = newStoryboard(n1,n2,init_scenario,story)
    list_of_pedestrians.append(ego_vehicle)
    list_of_pedestrians.append(bicycle)
    scenario = newScenario(n1,n2,list_of_pedestrians, storyboard, onto2.Town01)
    onto2.save(filename)
    return scenario

    
def newBicycle(n1,n2,asset_name,bounding_box):
    bicycle = onto2.Bicycle(n1+n2+"Bicycle")
    asset = newAsset(asset_name)
    bicycle.has_bounding_box.append(bounding_box)
    bicycle.has_asset.append(asset)
    return bicycle

# def changeWeather(n1,n2):
#     time_of_day = newTimeOfDay(n1,n2+"1310","true",2021,1,7,13,10,10)
#     road_condition = newRoadCondition(n1,n2+"1",0.5)
#     sun = newSun(n1,n2,1.3,0,0.8)
#     bounding_box = newBoundingBox(n1,n2+"weatherbb",10,10,10,10,10,10)
#     fog = newFog(n1,n2,10000,bounding_box)
#     precipitation = newPrecipitation(n1,n2+"dry",1,onto2.dry)
#     weather = newWeather(n1,n2+"foggy",onto2.free,sun,fog,precipitation)
#     environment = newEnvironment(n1,n2,time_of_day,weather,road_condition)
#     environment_action = newEnvironmentAction(n1,n2+"1",environment)
#     return environment_action

def newMisc(n1,n2,asset_name):
    misc = onto2.Misc(n1+n2+"misc")
    asset = newAsset(asset_name)
    misc.has_asset.append(asset)
    return misc

def main():
    n1 = "ped"
    n2 = "_"
    name_onto = "3"
    current_time = time.localtime()

    # # ##CutIn
    # filename_cut_in = f'risky_scenario_{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
    # cutInOntology("indiv","_",filename_cut_in)
    # print("Executing ConverterV03.py:")
    # command = "python3 ConverterV04.py " + filename_cut_in
    # print(command)
    # os.system("python3 ConverterV04.py " + filename_cut_in)


    #Fog
    # filename_into_fog = f'domain_shift_{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
    # intoFogOntology("indiv","_",filename_into_fog)
    # print("Executing ConverterV03.py:")
    # command = "python3 ConverterV03.py " + filename_into_fog
    # print(command)
    # os.system("python3 ConverterV04.py " + filename_into_fog)


    # #ManyPedestrians
    # filename_many_ped = f'collective_anomaly_{n2}_{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
    # command = "python3 ConverterV03.py " + filename_many_ped
    # print("running: " + command)
    # ManyPedestriansOntology("indiv","_",filename_many_ped)
    # os.system(command)


    # ##COCACOLA
    # filename_cola = f'_single_point_anomaly_{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
    # randomObjectOntology("indiv","_",filename_cola)
    # print("Executing ConverterV03.py:")
    # command = "python3 ConverterV03.py " + filename_cola
    # print(command)
    # os.system("python3 ConverterV04.py " + filename_cola)

    ##STREET SIGNS
    # filename_ss = f'_novel_scenario_{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
    # fallingSigns("indiv","_",filename_ss)
    # print("Executing ConverterV03.py:")
    # command = "python3 ConverterV03.py " + filename_ss
    # print(command)
    # os.system("python3 ConverterV04.py " + filename_ss)

    # #Bicycle
    filename_ss = f'anomalous_scenario{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
    BicycleOnOneWheelOntology("indiv","_",filename_ss)
    print("Executing ConverterV03.py:")
    command = "python3 ConverterV03.py " + filename_ss
    print(command)
    os.system("python3 ConverterV04.py " + filename_ss)

    #running into car
    # filename_ss = f'risky_scenario_running_into_car{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
    # runningIntoCar("indiv","_",filename_ss)
    # print("Executing ConverterV03.py:")
    # command = "python3 ConverterV03.py " + filename_ss
    # print(command)
    # # os.system("python3 /fzi/ids/xt274/no_backup/phd/generation/carla_test2.py" + " & python3 ConverterV04.py " + filename_ss)
    # os.system("python3 ConverterV04.py " + filename_ss)


    #merge 
    # nameoffile = f'merge_{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
    # test = testMerge("_test","_",nameoffile)
    # os.system("python3 ConverterV04.py " + nameoffile)
if __name__ == "__main__":
        main()
        print("done")
