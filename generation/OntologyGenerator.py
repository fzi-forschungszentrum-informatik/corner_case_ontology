from mimetypes import init
from owlready2 import *
import time
onto_path.append("./TemplateOntology.owl")
template_ontology = get_ontology("http://www.semanticweb.org/stefi/ontologies/2021/10/21/untitled-ontology-56")
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
    action = template_ontology.SpeedAction(n1 + n2 + "SpeedAction" )
    action.has_entity_ref.append(entity)
    action.has_target_speed.append(target_speed)
    action.has_transition_dynamics.append(transition_dynamics) 
    return action

def setCornerCase(scenario, corner_case_type):
    """
    Sets the corner case category of a scenario in the ontology
    
    Parameters:
   
    scenario - a scenario individual of the class Scenario, created with the method newScenario()
    
    corner_case_type - a constant individual within the ontology
    """
    scenario.has_corner_case.append(corner_case_type)
    return scenario
    
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
    action = template_ontology.TeleportAction(n1 + n2 + "TeleportAction")
    position = template_ontology.Position(n1 + n2 + "Position")
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
    action = template_ontology.TeleportAction(n1+n2+"TeleportActionRelative")
    position = template_ontology.RelativeObjectPosition(n1+n2+"RelativeObjectPosition")
    position.has_relative_position_object_reference.append(obj_ref)
    position.has_x.append(x)
    position.has_y.append(y)
    position.has_z.append(z)
    orientation = template_ontology.Orientation(n1+n2+"Orientation")
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
    maneuver = template_ontology.Maneuver(n1 + n2 + "Maneuver")
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
    mnvGrp = template_ontology.ManeuverGroup(n1 + n2 + "ManeuverGroup")
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
    act = template_ontology.Act(n1 + n2+"Act")
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
    story = template_ontology.Story(n1 + n2+"Story")
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
    event = template_ontology.Event(n1 + n2 + "Event")
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
    car = template_ontology.EgoVehicle(n1 + n2 + "EgoVehicle")
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
    car = template_ontology.Vehicle(n1 + n2 + "Car")
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
    condition = template_ontology.SimulationTimeCondition(n1 + n2 + "SimulationTimeCondition")
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
    condition_group = template_ontology.ConditionGroup(n1+n2+"ConditionGroup")
    for condition in conditions:
        condition_group.has_condition.append(condition)
    return condition_group

def newAsset(string):
    """
    Creates a new asset in the ontology from the given string.

    Returns a reference to the new Asset individual within the ontology
    """
    return template_ontology.Asset(string)
   
def newStartTrigger(condition_groups, n1 , n2):
    """
    Creates an individual of class StartTrigger in the new ontology with name "n1n2StartTrigger". 

    Parameters:

    condition_groups - an array with individuals of class ConditionGroup. Use newConditionGroup() to create those.

    n1, n2 - string used for the name of the individual 

    Returns a reference to the new StartTrigger individual within the ontology
    """
    startTrigger = template_ontology.StartTrigger(n1+n2+"StartTrigger")
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
    stopTrigger = template_ontology.StopTrigger(n1+n2+"StopTrigger")
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
    pedestrian = template_ontology.Pedestrian(n1+n2+"Pedestrian")
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
    return template_ontology.Town(name)


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
    scenario = template_ontology.Scenario(n1+n2+"Scenario")
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
    return_init = template_ontology.Init(n1 + n2+"Init")
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
    storyboard = template_ontology.Storyboard(n1+n2+"Storyboard")
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
    lane_action = template_ontology.RelativeLaneChangeAction(n1 + n2+"RelativeLaneChangeAction")
    lane_action.has_target_lane_id.append(lane_id)
    lane_action.has_entity_ref.append(entity)
    lane_action.has_offset.append(offset)
    lane_action.has_transition_dynamics.append(transition_dynamics)
    return lane_action


def newRelativeDistanceCondition(n1,n2, distance,condition_rule,distance_type,target_entity,entity_ref,delay,condition_edge):
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
    relative_distance_condition = template_ontology.RelativeDistanceCondition(n1 + n2 + "RelativeDistanceCondtion")
    relative_distance_condition.has_distance_value.append(distance)
    relative_distance_condition.has_target_reference.append(target_entity)
    relative_distance_condition.has_entity_ref.append(entity_ref)
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
    storyboard_element_state_condition = template_ontology.StoryboardElementStateCondition(n1+n2+"StoryboardElementStateCondition")
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
    time_of_day = template_ontology.TimeOfDay(n1 + n2 + "TimeOfDay")
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
    bounding_box = template_ontology.BoundingBox(n1 + n2 + "BoundingBox")
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
    fog = template_ontology.Fog(n1 + n2 + "Fog")
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
    precipitation = template_ontology.Precipitation(n1 + n2 + "Precipitation")
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
    sun = template_ontology.Sun(n1 + n2 + "Sun")
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
    weather = template_ontology.Weather(n1+n2+"Weather")
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
    road_condition = template_ontology.RoadCondition(n1+n2+"RoadCondition")
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
    environment = template_ontology.Environment(n1+n2+"Environment")
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
    environment_action = template_ontology.EnvironmentAction(n1 + n2 + "EnvironmentAction")
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
    transition_dynamics = template_ontology.TransitionDynamics(n1+n2+"TransitionDynamics")
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
    traveled_distance_condition = template_ontology.TraveledDistanceCondition(n1+n2+"TraveledDistanceCondition")
    traveled_distance_condition.has_entity_ref.append(entity_ref)
    traveled_distance_condition.has_traveled_distance_value.append(distance)
    traveled_distance_condition.has_condition_edge.append(condition_edge)
    return traveled_distance_condition

def newBicycle(n1,n2,asset_name,bounding_box):
    """
    Creates an individual of class Bicycle in the new ontology with name "n1n2Bicycle". 

    Parameters:

    n1, n2 - string used for the name of the individual. 

    asset_name - a string with the name of the asset or a direct refence to the ontology individual.

    bounding_box - an individual of the class BoundingBox. Use newBoundingBox() to create those.

    Returns a reference to the new Bicycle individual within the ontology.
    """

    bicycle = template_ontology.Bicycle(n1+n2+"Bicycle")
    asset = newAsset(asset_name)
    bicycle.has_bounding_box.append(bounding_box)
    bicycle.has_asset.append(asset)
    return bicycle

def newMisc(n1,n2,asset_name):
    """
    Creates an individual of class Pedestrian in the new ontology with name "n1n2Misc". 

    Parameters:

    n1, n2 - string used for the name of the individual. 

    asset_name - a string with the name of the asset or a direct refence to the ontology individual.

    Returns a reference to the new Misc individual within the ontology.
    """
    misc = template_ontology.Misc(n1+n2+"Misc")
    asset = newAsset(asset_name)
    misc.has_asset.append(asset)
    return misc


