import OntologyGenerator as OG
from owlready2 import *

template_ontology = get_ontology('TemplateOntology.owl').load()

##Cut In ontology Scenario
def cutInOntology(n1, n2, filename):
    """
    Creates a Scenario individual, where a very close cut in happens in front of the ego vehicle.

    Parameters:

    n1,n2 - string used for the name of the individuals.

    filename - string used for the name of the scenario

    Returns the Scenario individual, with correct property assertions.
    """

    transition_dynamics = OG.newTransitionDynamics(n1, n2 + "15", template_ontology.step, template_ontology.time, 0)
    # transition_dynamics1 = newTransitionDynamics(n1,n2+"16",onto2.step,onto2.distance,0)
    car_asset = OG.getCarAssets()[1]
    bounding_box_ego = OG.newBoundingBox(n1, n2 + "cut_in", 10, 10, 10, 10, 10, 10)
    ego_init_speed = 16.6666
    cut_in_speed = 22.222222
    ##Scenario
    ego_vehicle = template_ontology.ego_vehicle
    cut_in_vehicle = OG.newCar(n1, n2 + "_cutIn", car_asset, bounding_box_ego)
    entities = []
    entities.append(ego_vehicle)
    entities.append(cut_in_vehicle)
    ego_speed_action_init = OG.newSpeedAction(ego_vehicle, ego_init_speed, transition_dynamics, n1, n2 + "Ego")
    cut_in_speed_action_init = OG.newSpeedAction(cut_in_vehicle, cut_in_speed, transition_dynamics, n1, n2 + "CutIn")
    init_actions = [ego_speed_action_init, cut_in_speed_action_init]
    ego_teleport_action = OG.newTeleportActionWithPosition(ego_vehicle, 5, 20, 0, 50, n1, n2 + "Ego")
    init_actions.append(ego_teleport_action)
    cut_teleport_action = OG.newTeleportActionWithPosition(cut_in_vehicle, 5, 50, 0, 50, n1, n2 + "CutIn")
    init_actions.append(cut_teleport_action)
    init_actions.append(template_ontology.def_env_action)
    init_scenario = OG.newInit(n1, n2, init_actions)

    ##Story
    ##Actions
    lane_change_transition_dynamics = OG.newTransitionDynamics(n1, n2 + "Lane", template_ontology.cubic,
                                                            template_ontology.distance, 22)
    cut_in_lane_change_action_right = OG.newRelativeLaneChangeAction(n1, n2 + "CutIn", 1, cut_in_vehicle, 0,
                                                                  lane_change_transition_dynamics)
    cut_in_lane_change_action_left = OG.newRelativeLaneChangeAction(n1, n2 + "Left", -1, cut_in_vehicle, 0,
                                                                 lane_change_transition_dynamics)
    # Start triggers
    zero_sim_condition = OG.newSimulationCondition(0, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                                n2 + "zero")
    condtion_group_zero = OG.newConditionGroup([zero_sim_condition], n1, n2 + "Zero")
    start_trigger_zero = OG.newStartTrigger([condtion_group_zero], n1, n2 + "Zero")

    three_sim_condition = OG.newSimulationCondition(3, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                                 n2 + "3")
    condtion_group_three = OG.newConditionGroup([three_sim_condition], n1, n2 + "Three")
    start_trigger_three = OG.newStartTrigger([condtion_group_three], n1, n2 + "Three")
    relative_distance_condition = OG.newRelativeDistanceCondition(n1, n2, 15, template_ontology.lessThan,
                                                               template_ontology.cartesianDistance, cut_in_vehicle,
                                                               ego_vehicle, 0, template_ontology.rising)
    relative_distance_condition2 = OG.newRelativeDistanceCondition(n1, n2 + "Start", 17, template_ontology.lessThan,
                                                                template_ontology.cartesianDistance, cut_in_vehicle,
                                                                ego_vehicle, 0, template_ontology.rising)
    condition_group_distance = OG.newConditionGroup([relative_distance_condition], n1, n2 + "Distance")
    condition_group_distance2 = OG.newConditionGroup([relative_distance_condition2], n1, n2 + "Distance2")
    start_trigger_distance = OG.newStartTrigger([condition_group_distance], n1, n2 + "DistanceTrigger")
    start_trigger_distance2 = OG.newStartTrigger([condition_group_distance2], n1, n2 + "DistanceTrigger2")

    # Event
    event_cut_in_right = OG.newEvent([cut_in_lane_change_action_right], template_ontology.overwrite,
                                  start_trigger_distance, n1, n2 + "CutIn")
    # StoryboardElementCondition Start Trigger
    storyboard_element_state_condition_event1 = OG.newStoryboardElementStateCondition(n1, n2 + "cut_in_event",
                                                                                   template_ontology.completeState,
                                                                                   event_cut_in_right, 0,
                                                                                   template_ontology.none)
    traveled_distance_condition = OG.newTraveledDistanceCondition(n1, n2 + "70", cut_in_vehicle, 70,
                                                               template_ontology.none)
    conditionList = []
    conditionList.append(storyboard_element_state_condition_event1)
    conditionList.append(traveled_distance_condition)
    condtion_group_complete_state_and_distance = OG.newConditionGroup(conditionList, n1, n2 + "completeState")
    start_trigger_state = OG.newStartTrigger([condtion_group_complete_state_and_distance], n1, n2 + "StateTrigger")

    # Event2
    event_cut_in_left = OG.newEvent([cut_in_lane_change_action_left], template_ontology.overwrite, start_trigger_state, n1,
                                 n2 + "CutInLeft")

    # Longersimulation
    keep_longer_action = OG.newSpeedAction(ego_vehicle, ego_init_speed, transition_dynamics, n1, n2 + "longer")
    storyboard_element_state_condition_longer = OG.newStoryboardElementStateCondition(n1, n2 + "KeepLonger",
                                                                                   template_ontology.completeState,
                                                                                   event_cut_in_left, 5,
                                                                                   template_ontology.none)
    condtion_group_complete_state_longer = OG.newConditionGroup([storyboard_element_state_condition_longer], n1,
                                                             n2 + "completeState1")
    start_trigger_state_longer = OG.newStartTrigger([condtion_group_complete_state_longer], n1, n2 + "StateTrigger1")
    event_keep_longer = OG.newEvent([keep_longer_action], template_ontology.overwrite, start_trigger_state_longer, n1,
                                 n2 + "KeepLonger")

    maneuver = OG.newManeuver([event_cut_in_right, event_cut_in_left, event_keep_longer], n1, n2)
    maneuver_group = OG.newManeuverGroup([maneuver], cut_in_vehicle, n1, n2)

    ##stop triggers
    sim_condition_30 = OG.newSimulationCondition(30, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                              n2 + "30")
    condition_group_30 = OG.newConditionGroup([sim_condition_30], n1, n2 + "30")
    stop_trigger_30 = OG.newStopTrigger([condition_group_30], n1, n2 + "30")

    scenario_act = OG.newAct([maneuver_group], start_trigger_distance2, stop_trigger_30, n1, n2)
    story = OG.newStory([scenario_act], stop_trigger_30, n1, n2)
    storyboard = OG.newStoryboard(n1, n2, init_scenario, story)
    scenario = OG.newScenario(n1, n2, entities, storyboard, template_ontology.Town04)
    # ego_
    print("done cut_in")
    template_ontology.save(filename)
    return scenario


def fallingSigns(n1, n2, filename):
    """
    Creates a Scenario individual, where the ego vehicle is driving and suddenly 3 signs appear and start falling in front of the ego vehicle.

    Parameters:

    n1,n2 - string used for the name of the individuals.

    filename - string used for the name of the scenario

    Returns the Scenario individual, with correct property assertions.
    """

    ego_vehicle = template_ontology.ego_vehicle
    ego_init_speed = 7
    transition_dynamics = OG.newTransitionDynamics(n1, n2, template_ontology.step, template_ontology.distance, 15)
    ego_speed_action_init = OG.newSpeedAction(ego_vehicle, ego_init_speed, transition_dynamics, n1, n2 + "Ego")
    ego_teleport_action = OG.newTeleportActionWithPosition(ego_vehicle, -1, 0, 2, 1, n1, n2 + "Ego")
    streetsign1 = OG.newMisc(n1, n2 + "1", "static.prop.streetsign01")
    streetsign2 = OG.newMisc(n1, n2 + "2", "static.prop.streetsign01")
    streetsign3 = OG.newMisc(n1, n2 + "3", "static.prop.streetsign01")
    ss1_teleport_action = OG.newTeleportActionWithPosition(streetsign1, 1, 75, 0, 1, n1, n2 + "ss1_init")
    ss2_teleport_action = OG.newTeleportActionWithPosition(streetsign2, -2, 70, 0, 1, n1, n2 + "ss2_init")
    ss3_teleport_action = OG.newTeleportActionWithPosition(streetsign3, 1, 65, 0, 1, n1, n2 + "ss3_init")
    environment_action = template_ontology.def_env_action
    init_scenario = OG.newInit(n1, n2,
                            [ego_speed_action_init, ego_teleport_action, ss1_teleport_action, environment_action,
                             ss2_teleport_action, ss3_teleport_action])

    # StartTrigger
    zero_sim_condition = OG.newSimulationCondition(0, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                                n2 + "zero")
    condtion_group_zero = OG.newConditionGroup([zero_sim_condition], n1, n2 + "Zero")
    start_trigger_zero = OG.newStartTrigger([condtion_group_zero], n1, n2 + "Zero")

    # DistanceTrigger
    traveled_distance_condition = OG.newTraveledDistanceCondition(n1, n2 + "70", ego_vehicle, 50, template_ontology.none)
    condtion_group_distance = OG.newConditionGroup([traveled_distance_condition], n1, n2 + "Distance")
    start_trigger_distance = OG.newStartTrigger([condtion_group_distance], n1, n2 + "Distance")

    # StopTrigger
    sim_condition_30 = OG.newSimulationCondition(30, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                              n2 + "30")
    condtion_group_30 = OG.newConditionGroup([sim_condition_30], n1, n2 + "30")
    stop_trigger_30 = OG.newStopTrigger([condtion_group_30], n1, n2 + "30")

    # bbq teleport action
    bbq_teleport_action_story = OG.newTeleportActionWithRelativePositionAndOrientation(n1, n2 + "ss1", streetsign1, 0, 0,
                                                                                    0, 0.0, -0.5, 0)
    event1 = OG.newEvent([bbq_teleport_action_story], template_ontology.overwrite, start_trigger_distance, n1,
                      n2 + "ss1_distance")
    maneuver1 = OG.newManeuver([event1], n1, n2 + "ss1")
    mg1 = OG.newManeuverGroup([maneuver1], streetsign1, n1, n2 + "ss1")
    # ss2
    bbq_teleport_action_story2 = OG.newTeleportActionWithRelativePositionAndOrientation(n1, n2 + "ss2", streetsign2, 0, 0,
                                                                                     0, 0.0, -0.5, 0)
    event2 = OG.newEvent([bbq_teleport_action_story2], template_ontology.overwrite, start_trigger_distance, n1,
                      n2 + "ss2_distance")
    maneuver2 = OG.newManeuver([event2], n1, n2 + "ss2")
    mg2 = OG.newManeuverGroup([maneuver2], streetsign2, n1, n2 + "ss2")
    ##ss3
    bbq_teleport_action_story3 = OG.newTeleportActionWithRelativePositionAndOrientation(n1, n2 + "ss3", streetsign3, 0, 0,
                                                                                     0, 0.0, -0.5, 0)
    event3 = OG.newEvent([bbq_teleport_action_story3], template_ontology.overwrite, start_trigger_distance, n1,
                      n2 + "ss3_distance")
    maneuver3 = OG.newManeuver([event3], n1, n2 + "ss3")
    mg3 = OG.newManeuverGroup([maneuver3], streetsign3, n1, n2 + "ss3")

    # ego speed
    keep_longer_action = OG.newSpeedAction(ego_vehicle, 5, transition_dynamics, n1, n2 + "longer")
    storyboard_element_state_condition_event1 = OG.newStoryboardElementStateCondition(n1, n2 + "speedchange",
                                                                                   template_ontology.completeState,
                                                                                   event1, 10, template_ontology.none)
    condtion_group_complete_state = OG.newConditionGroup([storyboard_element_state_condition_event1], n1,
                                                      n2 + "completeState")
    start_trigger_state = OG.newStartTrigger([condtion_group_complete_state], n1, n2 + "StateTrigger")
    event_keep_longer = OG.newEvent([keep_longer_action], template_ontology.overwrite, start_trigger_state, n1,
                                 n2 + "KeepLonger")
    maneuver_ego = OG.newManeuver([event_keep_longer], n1, n2)
    maneuver_group = OG.newManeuverGroup([maneuver_ego], ego_vehicle, n1, n2)

    # Act and rest of the scenario
    scenario_act = OG.newAct([mg1, maneuver_group, mg2, mg3], start_trigger_zero, stop_trigger_30, n1, n2 + "1")
    story = OG.newStory([scenario_act], stop_trigger_30, n1, n2)
    storyboard = OG.newStoryboard(n1, n2, init_scenario, story)
    scenario = OG.newScenario(n1, n2, [ego_vehicle, streetsign1, streetsign2, streetsign3], storyboard,
                           template_ontology.Town04)
    template_ontology.save(filename)
    return scenario
    ##Fog Ontology scenario


def intoFogOntology(n1, n2, filename):
    """
    Creates a Scenario individual, where the ego vehicle is driving, and after some time a dense fog appears.

    Parameters:

    n1,n2 - string used for the name of the individuals.

    filename - string used for the name of the scenario

    Returns the Scenario individual, with correct property assertions.
    """

    ego_init_speed = 15
    bounding_box_ego = OG.newBoundingBox(n1, n2 + "Ego", 10, 10, 10, 10, 10, 10)
    car_assets = OG.getCarAssets()

    ego_vehicle = template_ontology.ego_vehicle
    transition_dynamics = OG.newTransitionDynamics(n1, n2, template_ontology.step, template_ontology.distance, 15)
    ego_teleport_action = OG.newTeleportActionWithPosition(ego_vehicle, 5, 50, 0, 48, n1, n2 + "Ego")
    ego_speed_action_init = OG.newSpeedAction(ego_vehicle, ego_init_speed, transition_dynamics, n1, n2 + "Ego")
    init_weather = template_ontology.def_env_action
    init_scenario = OG.newInit(n1, n2, [ego_speed_action_init, ego_teleport_action, init_weather])
    # Environment Action
    time_of_day = OG.newTimeOfDay(n1, n2 + "1200", "true", 2021, 1, 7, 13, 10, 10)
    road_condition = OG.newRoadCondition(n1, n2 + "1", 1)
    sun = OG.newSun(n1, n2, 1.3, 0, 0.8)
    bounding_box = OG.newBoundingBox(n1, n2 + "Fog_bb", 10, 10, 10, 10, 10, 10)
    fog = OG.newFog(n1, n2, 50, bounding_box)
    precipitation = OG.newPrecipitation(n1, n2 + "dry", 1, template_ontology.dry)
    weather = OG.newWeather(n1, n2 + "foggy", template_ontology.free, sun, fog, precipitation)
    environment = OG.newEnvironment(n1, n2, time_of_day, weather, road_condition)
    environment_action = OG.newEnvironmentAction(n1, n2 + "1", environment)

    # Triggers
    zero_sim_condition = OG.newSimulationCondition(0, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                                n2 + "zero")
    condtion_group_zero = OG.newConditionGroup([zero_sim_condition], n1, n2 + "Zero")
    start_trigger_zero = OG.newStartTrigger([condtion_group_zero], n1, n2 + "Zero")

    traveled_distance_condition = OG.newTraveledDistanceCondition(n1, n2 + "70", ego_vehicle, 70, template_ontology.none)
    condtion_group_distance = OG.newConditionGroup([traveled_distance_condition], n1, n2 + "Distance")
    start_trigger_distance = OG.newStartTrigger([condtion_group_distance], n1, n2 + "Distance")

    event_weather_change = OG.newEvent([environment_action], template_ontology.overwrite, start_trigger_distance, n1,
                                    n2 + "weather_change")
    ##Keep the scenario going on for a bit longer
    keep_longer_action = OG.newSpeedAction(ego_vehicle, 5, transition_dynamics, n1, n2 + "longer")
    storyboard_element_state_condition_event1 = OG.newStoryboardElementStateCondition(n1, n2 + "weatherchange",
                                                                                   template_ontology.completeState,
                                                                                   event_weather_change, 5,
                                                                                   template_ontology.none)
    condtion_group_complete_state = OG.newConditionGroup([storyboard_element_state_condition_event1], n1,
                                                      n2 + "completeState")
    start_trigger_state = OG.newStartTrigger([condtion_group_complete_state], n1, n2 + "StateTrigger")
    event_keep_longer = OG.newEvent([keep_longer_action], template_ontology.overwrite, start_trigger_state, n1,
                                 n2 + "KeepLonger")
    ##
    maneuver = OG.newManeuver([event_weather_change, event_keep_longer], n1, n2)
    maneuver_group = OG.newManeuverGroup([maneuver], ego_vehicle, n1, n2)

    ##stop triggers
    sim_condition_30 = OG.newSimulationCondition(30, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                              n2 + "30")
    condtion_group_30 = OG.newConditionGroup([sim_condition_30], n1, n2 + "30")
    stop_trigger_30 = OG.newStopTrigger([condtion_group_30], n1, n2 + "30")

    scenario_act = OG.newAct([maneuver_group], start_trigger_zero, stop_trigger_30, n1, n2)
    story = OG.newStory([scenario_act], stop_trigger_30, n1, n2)
    storyboard = OG.newStoryboard(n1, n2, init_scenario, story)
    scenario = OG.newScenario(n1, n2, [ego_vehicle], storyboard, template_ontology.Town04)

    print("done environment")
    template_ontology.save(filename)
    return scenario


def ManyPedestriansOntology(n1, n2, filename):
    """
    Creates a Scenario individual, where the ego vehicle is driving behind "n" amount of pedestrians. After some time, the pedestrians start running.

    Parameters:

    n1,n2 - string used for the name of the individuals.

    filename - string used for the name of the scenario

    Returns the Scenario individual, with correct property assertions.
    """

    transition_dynamics = OG.newTransitionDynamics(n1, n2, template_ontology.step, template_ontology.distance, 15)
    scenarioTown = template_ontology.Town01
    amountPedestrians = 10
    newScenario = template_ontology.Scenario(n1 + n2 + "Scenario")
    newStoryboard = template_ontology.Storyboard(n1 + n2 + "Storyboard")
    newScenario.has_storyboard.append(newStoryboard)
    newScenario.has_town.append(scenarioTown)
    bounding_box = OG.newBoundingBox(n1, n2 + "bb", 10, 10, 10, 10, 10, 10)
    # entities
    pedestrian_assets = OG.getPedestrianAssets()
    list_of_pedestrians = []
    # creating amountPedestrians pedestrians
    for i in range(amountPedestrians):
        ped = OG.newPedestrian(n1, str(i), pedestrian_assets[i], bounding_box)
        list_of_pedestrians.append(ped)
    for i in range(amountPedestrians):
        ped = OG.newPedestrian(n1, str(i), pedestrian_assets[i], bounding_box)
        newScenario.has_entity.append(ped)
    # story
    story = template_ontology.Story(n1 + n2 + "Story")
    newStoryboard.has_story.append(story)

    ego_vehicle = template_ontology.ego_vehicle
    newScenario.has_entity.append(ego_vehicle)
    # init
    new_init = template_ontology.Init(n1 + n2 + "Init1")
    # Ego Vehicle Actions
    actionTeleportEgo = OG.newTeleportActionWithPosition(ego_vehicle, -1, 0, 0, 1, n1, n2 + "EgoTeleport")
    actionSpeedEgo = OG.newSpeedAction(ego_vehicle, 3, transition_dynamics, n1, n2 + "EgoSpeed")
    new_init.has_init_action.append(actionSpeedEgo)
    new_init.has_init_action.append(actionTeleportEgo)
    weatheraction = template_ontology.def_env_action
    new_init.has_init_action.append(weatheraction)
    newStoryboard.has_init.append(new_init)
    maneuver_groups = []
    # Add amountPedestrians pedestrians in the ontology
    s_jump = 20
    for i in range(amountPedestrians):
        actions = []
        events = []
        place = 5
        pedestrian_assets = OG.getPedestrianAssets()
        pedestrian = OG.newPedestrian(n1, str(i), pedestrian_assets[i], bounding_box)
        if i % 2 == 0:
            offset = -1
        else:
            offset = 0
        actionTeleport = OG.newTeleportActionWithPosition(pedestrian, -1, s_jump, offset, 1, n1, str(i))
        actionSpeed = OG.newSpeedAction(pedestrian, 2, transition_dynamics, n1, str(i))
        new_init.has_init_action.append(actionTeleport)
        new_init.has_init_action.append(actionSpeed)
        action3 = OG.newSpeedAction(pedestrian, 8, transition_dynamics, n1 + "_0_", str(i))
        actions.append(action3)
        zero_sim_condition = OG.newSimulationCondition(5, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                                    n2 + "zero")
        condtion_group_zero = OG.newConditionGroup([zero_sim_condition], n1, n2 + "Zero")
        start_trigger_zero = OG.newStartTrigger([condtion_group_zero], n1, n2 + "Zero")
        event1 = OG.newEvent(actions, template_ontology.overwrite, start_trigger_zero, n1, str(i))
        events.append(event1)
        maneuver1 = OG.newManeuver(events, n1, str(i))
        maneuverGroup1 = OG.newManeuverGroup([maneuver1], pedestrian, n1, str(i))
        maneuver_groups.append(maneuverGroup1)
        s_jump += 4
    sim_condition_30 = OG.newSimulationCondition(30, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                              n2 + "30")
    condtion_group_30 = OG.newConditionGroup([sim_condition_30], n1, n2 + "30")
    stop_trigger_30 = OG.newStopTrigger([condtion_group_30], n1, n2 + "30")
    zero_sim_condition = OG.newSimulationCondition(5, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                                n2 + "zero")
    condtion_group_zero = OG.newConditionGroup([zero_sim_condition], n1, n2 + "Zero")
    start_trigger_zero = OG.newStartTrigger([condtion_group_zero], n1, n2 + "Zero")

    act = OG.newAct(maneuver_groups, start_trigger_zero, stop_trigger_30, n1, n2)
    story.has_act.append(act)
    template_ontology.save(filename)


def BicycleOnOneWheelOntology(n1, n2, filename):
    """
    Creates a Scenario individual, where the ego vehicle is just driving and in the opposite lane, a cyclist starts executing strange manuevers.

    Parameters:

    n1,n2 - string used for the name of the individuals.

    filename - string used for the name of the scenario

    Returns the Scenario individual, with correct property assertions.
    """

    ego_init_speed = 5
    bicycle_speed = 8
    bounding_box_bike = OG.newBoundingBox(n1, n2 + "Ego", 10, 10, 10, 10, 10, 10)
    ##Scenario
    transition_dynamics = OG.newTransitionDynamics(n1, n2, template_ontology.step, template_ontology.distance, 15)
    ego_vehicle = template_ontology.ego_vehicle
    bicycle = OG.newBicycle(n1, n2 + "bike", OG.getBicycleAssets()[0], bounding_box_bike)
    entities = []
    entities.append(ego_vehicle)
    entities.append(bicycle)
    # Init Actions and init
    ego_speed_action_init = OG.newSpeedAction(ego_vehicle, ego_init_speed, transition_dynamics, n1, n2 + "Ego")
    bike_speed_action_init = OG.newSpeedAction(bicycle, bicycle_speed, transition_dynamics, n1, n2 + "bicycle")
    init_actions = [ego_speed_action_init, bike_speed_action_init]
    ego_teleport_action = OG.newTeleportActionWithPosition(ego_vehicle, -1, 0, 0, 1, n1, n2 + "Ego")
    init_actions.append(ego_teleport_action)
    bike_teleport_action = OG.newTeleportActionWithPosition(bicycle, 1, 80, 0, 1, n1, n2 + "bicycle")
    init_actions.append(bike_teleport_action)
    init_actions.append(template_ontology.def_env_action)
    init_scenario = OG.newInit(n1, n2, init_actions)
    # Triggers
    zero_sim_condition = OG.newSimulationCondition(0, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                                n2 + "zero")
    condtion_group_zero = OG.newConditionGroup([zero_sim_condition], n1, n2 + "Zero")
    start_trigger_zero = OG.newStartTrigger([condtion_group_zero], n1, n2 + "Zero")

    traveled_distance_condition = OG.newTraveledDistanceCondition(n1, n2 + "10", bicycle, 20, template_ontology.rising)
    traveled_condition_group = OG.newConditionGroup([traveled_distance_condition], n1, n2 + "travel")
    start_trigger_distance = OG.newStartTrigger([traveled_condition_group], n1, n2 + "travel")

    # StopTrigger
    sim_condition_30 = OG.newSimulationCondition(30, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                              n2 + "30")
    condtion_group_30 = OG.newConditionGroup([sim_condition_30], n1, n2 + "30")
    stop_trigger_30 = OG.newStopTrigger([condtion_group_30], n1, n2 + "30")

    # Event
    bicycle_teleport_action_story = OG.newTeleportActionWithRelativePositionAndOrientation(n1, n2 + "bicycle1", bicycle, 0,
                                                                                        0, 0.7, 0.7, 0, 0.7)
    event1 = OG.newEvent([bicycle_teleport_action_story], template_ontology.overwrite, start_trigger_distance, n1,
                      n2 + "bicycle_distance")
    maneuver1 = OG.newManeuver([event1], n1, n2 + "bicycle1")
    mg1 = OG.newManeuverGroup([maneuver1], bicycle, n1, n2 + "bicycle1")

    # keeplonger
    keep_longer_action = OG.newSpeedAction(ego_vehicle, 5, transition_dynamics, n1, n2 + "longer")
    storyboard_element_state_condition_event1 = OG.newStoryboardElementStateCondition(n1, n2 + "speedchange",
                                                                                   template_ontology.completeState,
                                                                                   event1, 1, template_ontology.none)
    condtion_group_complete_state = OG.newConditionGroup([storyboard_element_state_condition_event1], n1,
                                                      n2 + "completeState")
    start_trigger_state = OG.newStartTrigger([condtion_group_complete_state], n1, n2 + "StateTrigger")
    event_keep_longer = OG.newEvent([keep_longer_action], template_ontology.overwrite, start_trigger_state, n1,
                                 n2 + "KeepLonger")
    maneuver_ego = OG.newManeuver([event_keep_longer], n1, n2)
    maneuver_group = OG.newManeuverGroup([maneuver_ego], ego_vehicle, n1, n2)

    # rest
    scenario_act = OG.newAct([mg1, maneuver_group], start_trigger_zero, stop_trigger_30, n1, n2 + "1")
    story = OG.newStory([scenario_act], stop_trigger_30, n1, n2)
    storyboard = OG.newStoryboard(n1, n2, init_scenario, story)
    scenario = OG.newScenario(n1, n2, [ego_vehicle, bicycle], storyboard, template_ontology.Town01)
    template_ontology.save(filename)
    return scenario


def randomObjectOntology(n1, n2, filename):
    """
    Creates a Scenario individual, where the ego vehicle is driving and suddenly an unknown objects appears, that falls very close to the ego vehicle.
    Parameters:

    n1,n2 - string used for the name of the individuals.

    filename - string used for the name of the scenario

    Returns the Scenario individual, with correct property assertions.
    """

    ego_vehicle = template_ontology.ego_vehicle
    ego_init_speed = 10
    transition_dynamics = OG.newTransitionDynamics(n1, n2, template_ontology.step, template_ontology.distance, 15)
    ego_speed_action_init = OG.newSpeedAction(ego_vehicle, ego_init_speed, transition_dynamics, n1, n2 + "Ego")
    ego_teleport_action = OG.newTeleportActionWithPosition(ego_vehicle, -1, 0, 2, 1, n1, n2 + "Ego")
    barbeque1 = OG.newMisc(n1, n2, "static.prop.vendingmachine")
    barbeque_teleport_action = OG.newTeleportActionWithPosition(barbeque1, 1, 75, 0, 1, n1, n2 + "bbq_init")
    environment_action = template_ontology.def_env_action
    init_scenario = OG.newInit(n1, n2,
                            [ego_speed_action_init, ego_teleport_action, barbeque_teleport_action, environment_action])

    # StartTrigger
    zero_sim_condition = OG.newSimulationCondition(0, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                                n2 + "zero")
    condtion_group_zero = OG.newConditionGroup([zero_sim_condition], n1, n2 + "Zero")
    start_trigger_zero = OG.newStartTrigger([condtion_group_zero], n1, n2 + "Zero")

    # DistanceTrigger
    traveled_distance_condition = OG.newTraveledDistanceCondition(n1, n2 + "70", ego_vehicle, 50, template_ontology.none)
    condtion_group_distance = OG.newConditionGroup([traveled_distance_condition], n1, n2 + "Distance")
    start_trigger_distance = OG.newStartTrigger([condtion_group_distance], n1, n2 + "Distance")

    # StopTrigger
    sim_condition_30 = OG.newSimulationCondition(30, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                              n2 + "30")
    condtion_group_30 = OG.newConditionGroup([sim_condition_30], n1, n2 + "30")
    stop_trigger_30 = OG.newStopTrigger([condtion_group_30], n1, n2 + "30")

    # bbq teleport action
    bbq_teleport_action_story = OG.newTeleportActionWithRelativePositionAndOrientation(n1, n2, barbeque1, 0, 0, 0, 0.0,
                                                                                    -0.5, 0)
    event1 = OG.newEvent([bbq_teleport_action_story], template_ontology.overwrite, start_trigger_distance, n1,
                      n2 + "bbq_distance")
    maneuver1 = OG.newManeuver([event1], n1, n2 + "bbq")
    mg1 = OG.newManeuverGroup([maneuver1], barbeque1, n1, n2 + "bbq")

    # ego speed
    keep_longer_action = OG.newSpeedAction(ego_vehicle, 5, transition_dynamics, n1, n2 + "longer")
    storyboard_element_state_condition_event1 = OG.newStoryboardElementStateCondition(n1, n2 + "speedchange",
                                                                                   template_ontology.completeState,
                                                                                   event1, 20, template_ontology.none)
    condtion_group_complete_state = OG.newConditionGroup([storyboard_element_state_condition_event1], n1,
                                                      n2 + "completeState")
    start_trigger_state = OG.newStartTrigger([condtion_group_complete_state], n1, n2 + "StateTrigger")
    event_keep_longer = OG.newEvent([keep_longer_action], template_ontology.overwrite, start_trigger_state, n1,
                                 n2 + "KeepLonger")
    maneuver_ego = OG.newManeuver([event_keep_longer], n1, n2)
    maneuver_group = OG.newManeuverGroup([maneuver_ego], ego_vehicle, n1, n2)

    # Act and rest of the scenario
    scenario_act = OG.newAct([mg1, maneuver_group], start_trigger_zero, stop_trigger_30, n1, n2 + "1")
    story = OG.newStory([scenario_act], stop_trigger_30, n1, n2)
    storyboard = OG.newStoryboard(n1, n2, init_scenario, story)
    scenario = OG.newScenario(n1, n2, [ego_vehicle, barbeque1], storyboard, template_ontology.Town04)
    template_ontology.save(filename)
    return scenario


def runningIntoCar(n1, n2, filename):
    """
    Creates a Scenario individual, where the ego vehicle is driving and a pedestrian suddenly runs in front of the ego vehicle.

    Parameters:

    n1,n2 - string used for the name of the individuals.

    filename - string used for the name of the scenario

    Returns the Scenario individual, with correct property assertions.
    """

    bounding_box_object = OG.newBoundingBox(n1, n2 + "Ego", 10, 10, 10, 10, 10, 10)
    ##Scenario
    transition_dynamics = OG.newTransitionDynamics(n1, n2, template_ontology.step, template_ontology.distance, 15)
    ego_vehicle = template_ontology.ego_vehicle
    pedestrian = OG.newPedestrian(n1, n2 + "pedestrian", OG.getPedestrianAssets()[0], bounding_box_object)
    ego_teleport_action = OG.newTeleportActionWithPosition(ego_vehicle, -1, 0, 0, 1, n1, n2 + "Ego")
    ped_teleport_action = OG.newTeleportActionWithPosition(pedestrian, 2, 40, 2, 1, n1, n2 + "pedestrian")
    ego_speed_action_init = OG.newSpeedAction(ego_vehicle, 5, transition_dynamics, n1, n2 + "Ego")
    weather_action = template_ontology.def_env_action
    init_scenario = OG.newInit(n1, n2, [ego_teleport_action, ped_teleport_action, ego_speed_action_init, weather_action])

    # triggers
    zero_sim_condition = OG.newSimulationCondition(0, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                                n2 + "zero")
    condtion_group_zero = OG.newConditionGroup([zero_sim_condition], n1, n2 + "Zero")
    start_trigger_zero = OG.newStartTrigger([condtion_group_zero], n1, n2 + "Zero")

    relative_distance_condition = OG.newRelativeDistanceCondition(n1, n2, 17, template_ontology.lessThan,
                                                               template_ontology.cartesianDistance,
                                                               template_ontology.ego_vehicle, pedestrian, 0,
                                                               template_ontology.rising)
    condition_group_distance = OG.newConditionGroup([relative_distance_condition], n1, n2 + "Distance")
    start_trigger_distance = OG.newStartTrigger([condition_group_distance], n1, n2 + "DistanceTrigger")

    # StopTrigger
    sim_condition_30 = OG.newSimulationCondition(30, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                              n2 + "30")
    condtion_group_30 = OG.newConditionGroup([sim_condition_30], n1, n2 + "30")
    stop_trigger_30 = OG.newStopTrigger([condtion_group_30], n1, n2 + "30")

    ## Start running event
    pedestrian_teleport_action_story = OG.newTeleportActionWithRelativePositionAndOrientation(n1, n2 + "ss1", pedestrian,
                                                                                           0, 0, 0, 0, 0, 1.5)
    pedestrian_speed_action = OG.newSpeedAction(pedestrian, 7, transition_dynamics, n1, n2 + "pedestrian")
    event1 = OG.newEvent([pedestrian_teleport_action_story, pedestrian_speed_action], template_ontology.overwrite,
                      start_trigger_distance, n1, n2 + "bbq_distance")
    maneuver1 = OG.newManeuver([event1], n1, n2 + "pedestrian1")
    mg1 = OG.newManeuverGroup([maneuver1], pedestrian, n1, n2 + "pedestrian2")

    # keeplonger
    keep_longer_action = OG.newSpeedAction(ego_vehicle, 5, transition_dynamics, n1, n2 + "longer")
    storyboard_element_state_condition_event1 = OG.newStoryboardElementStateCondition(n1, n2 + "speedchange",
                                                                                   template_ontology.completeState,
                                                                                   event1, 1, template_ontology.none)
    condtion_group_complete_state = OG.newConditionGroup([storyboard_element_state_condition_event1], n1,
                                                      n2 + "completeState")
    start_trigger_state = OG.newStartTrigger([condtion_group_complete_state], n1, n2 + "StateTrigger")
    event_keep_longer = OG.newEvent([keep_longer_action], template_ontology.overwrite, start_trigger_state, n1,
                                 n2 + "KeepLonger")
    maneuver_ego = OG.newManeuver([event_keep_longer], n1, n2)
    maneuver_group = OG.newManeuverGroup([maneuver_ego], ego_vehicle, n1, n2)

    # rest
    scenario_act = OG.newAct([mg1, maneuver_group], start_trigger_zero, stop_trigger_30, n1, n2 + "1")
    story = OG.newStory([scenario_act], stop_trigger_30, n1, n2)
    storyboard = OG.newStoryboard(n1, n2, init_scenario, story)
    scenario = OG.newScenario(n1, n2, [ego_vehicle, pedestrian], storyboard, template_ontology.Town01)
    template_ontology.save(filename)
    return scenario


def testMerge(n1, n2, filename):
    """
    Creates a Scenario individual, where the bicycle and manypedestrians Scenarios are merged.

    Parameters:

    n1,n2 - string used for the name of the individuals.

    filename - string used for the name of the scenario

    Returns the Scenario individual, with correct property assertions.
    """

    ego_init_speed = 8
    bicycle_speed = 8
    bounding_box_bike = OG.newBoundingBox(n1, n2 + "Ego", 10, 10, 10, 10, 10, 10)
    ##Scenario
    transition_dynamics = OG.newTransitionDynamics(n1, n2, template_ontology.step, template_ontology.distance, 15)
    ego_vehicle = template_ontology.ego_vehicle
    bicycle = OG.newBicycle(n1, n2 + "bike", OG.getBicycleAssets()[0], bounding_box_bike)
    entities = []
    entities.append(ego_vehicle)
    entities.append(bicycle)
    # Init Actions and init
    ego_speed_action_init = OG.newSpeedAction(ego_vehicle, ego_init_speed, transition_dynamics, n1, n2 + "Ego")
    bike_speed_action_init = OG.newSpeedAction(bicycle, bicycle_speed, transition_dynamics, n1, n2 + "bicycle")
    weather_change = template_ontology.def_env_action
    init_actions = [ego_speed_action_init, bike_speed_action_init, weather_change]
    ego_teleport_action = OG.newTeleportActionWithPosition(ego_vehicle, -1, 0, 0, 1, n1, n2 + "Ego")
    init_actions.append(ego_teleport_action)
    bike_teleport_action = OG.newTeleportActionWithPosition(bicycle, 1, 80, 0, 1, n1, n2 + "bicycle")
    init_actions.append(bike_teleport_action)
    # Triggers
    zero_sim_condition = OG.newSimulationCondition(0, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                                n2 + "zero")
    condtion_group_zero = OG.newConditionGroup([zero_sim_condition], n1, n2 + "Zero")
    start_trigger_zero = OG.newStartTrigger([condtion_group_zero], n1, n2 + "Zero")

    traveled_distance_condition = OG.newTraveledDistanceCondition(n1, n2 + "10", bicycle, 20, template_ontology.rising)
    traveled_condition_group = OG.newConditionGroup([traveled_distance_condition], n1, n2 + "travel")
    start_trigger_distance = OG.newStartTrigger([traveled_condition_group], n1, n2 + "travel")

    # StopTrigger
    sim_condition_30 = OG.newSimulationCondition(30, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                              n2 + "30")
    condtion_group_30 = OG.newConditionGroup([sim_condition_30], n1, n2 + "30")
    stop_trigger_30 = OG.newStopTrigger([condtion_group_30], n1, n2 + "30")

    # Event
    bicycle_teleport_action_story = OG.newTeleportActionWithRelativePositionAndOrientation(n1, n2 + "ss1", bicycle, 0, 0,
                                                                                        0.7, 0.7, 0, 0.7)
    event1 = OG.newEvent([bicycle_teleport_action_story], template_ontology.overwrite, start_trigger_distance, n1,
                      n2 + "bbq_distance")
    maneuver1 = OG.newManeuver([event1], n1, n2 + "ss1")
    mg1 = OG.newManeuverGroup([maneuver1], bicycle, n1, n2 + "ss1")

    # keeplonger
    keep_longer_action = OG.newSpeedAction(ego_vehicle, 5, transition_dynamics, n1, n2 + "longer")
    storyboard_element_state_condition_event1 = OG.newStoryboardElementStateCondition(n1, n2 + "speedchange",
                                                                                   template_ontology.completeState,
                                                                                   event1, 1, template_ontology.none)
    condtion_group_complete_state = OG.newConditionGroup([storyboard_element_state_condition_event1], n1,
                                                      n2 + "completeState")
    start_trigger_state = OG.newStartTrigger([condtion_group_complete_state], n1, n2 + "StateTrigger")
    event_keep_longer = OG.newEvent([keep_longer_action], template_ontology.overwrite, start_trigger_state, n1,
                                 n2 + "KeepLonger")
    maneuver_ego = OG.newManeuver([event_keep_longer], n1, n2)
    maneuver_group = OG.newManeuverGroup([maneuver_ego], ego_vehicle, n1, n2)
    maneuver_groups = []
    s_jump = 20
    amountPedestrians = 10
    list_of_pedestrians = []
    for i in range(amountPedestrians):
        actions = []
        events = []
        place = 5
        pedestrian_assets = OG.getPedestrianAssets()
        pedestrian = OG.newPedestrian(n1, str(i), pedestrian_assets[i], bounding_box_bike)
        list_of_pedestrians.append(pedestrian)
        if i % 2 == 0:
            offset = -1
        else:
            offset = 0
        actionTeleport = OG.newTeleportActionWithPosition(pedestrian, -1, s_jump, offset, 1, n1, str(i))
        actionSpeed = OG.newSpeedAction(pedestrian, 2, transition_dynamics, n1, str(i))
        init_actions.append(actionTeleport)
        init_actions.append(actionSpeed)
        action3 = OG.newSpeedAction(pedestrian, 8, transition_dynamics, n1 + "_0_", str(i))
        # action4 = newTeleportActionWithPosition(pedestrian, 2, 22, 0, i+1, n1 + n3, str(i))
        actions.append(action3)
        # actions.append(action4)
        zero_sim_condition = OG.newSimulationCondition(5, 0, template_ontology.rising, template_ontology.greaterThan, n1,
                                                    n2 + "zero")
        condtion_group_zero = OG.newConditionGroup([zero_sim_condition], n1, n2 + "Zero")
        start_trigger_zero = OG.newStartTrigger([condtion_group_zero], n1, n2 + "Zero")
        event1 = OG.newEvent(actions, template_ontology.overwrite, start_trigger_zero, n1, str(i))
        events.append(event1)
        maneuver1 = OG.newManeuver(events, n1, str(i))
        maneuverGroup1 = OG.newManeuverGroup([maneuver1], pedestrian, n1, str(i))
        maneuver_groups.append(maneuverGroup1)
        s_jump += 4
    # rest
    init_scenario = OG.newInit(n1, n2, init_actions)
    maneuver_groups.append(mg1)
    maneuver_groups.append(maneuver_group)
    scenario_act = OG.newAct(maneuver_groups, start_trigger_zero, stop_trigger_30, n1, n2 + "1")
    story = OG.newStory([scenario_act], stop_trigger_30, n1, n2)
    storyboard = OG.newStoryboard(n1, n2, init_scenario, story)
    list_of_pedestrians.append(ego_vehicle)
    list_of_pedestrians.append(bicycle)
    scenario = OG.newScenario(n1, n2, list_of_pedestrians, storyboard, template_ontology.Town01)
    template_ontology.save(filename)
    return scenario

def main():
    n1 = "ped"
    n2 = "_"
    name_onto = "3"
    current_time = time.localtime()

    # ##CutIn
#    filename_cut_in  = f'risky_scenario_{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
#    cutInOntology("indiv","_",filename_cut_in)
#    print("Executing Onto2OpenSCENARIO.py:")
#    command = "python3 Onto2OpenSCENARIO.py " + filename_cut_in
#    print(command)
#    os.system("python3 Onto2OpenSCENARIO.py " + filename_cut_in)


    #Fog
#    filename_into_fog = f'domain_shift_{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
#    intoFogOntology("indiv","_",filename_into_fog)
#    print("Executing Onto2OpenSCENARIO.py:")
#    command = "python3 Onto2OpenSCENARIO.py " + filename_into_fog
#    print(command)
#     os.system("python3 Onto2OpenSCENARIO.py " + filename_into_fog)


    # #ManyPedestrians
#    filename_many_ped = f'collective_anomaly_{n2}_{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
    # command = "python3 Onto2OpenSCENARIO.py " + filename_many_ped
    # print("running: " + command)
#    ManyPedestriansOntology("indiv","_",filename_many_ped)
    # os.system(command)


    # # ##COCACOLA
#    filename_cola = f'_single_point_anomaly_{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
#    randomObjectOntology("indiv","_",filename_cola)
    # print("Executing Onto2OpenSCENARIO.py:")
    # command = "python3 Onto2OpenSCENARIO.py " + filename_cola
    # print(command)
    # os.system("python3 Onto2OpenSCENARIO.py " + filename_cola)

    ##STREET SIGNS
#    filename_ss = f'_novel_scenario_{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
#    fallingSigns("indiv","_",filename_ss)
    # print("Executing Onto2OpenSCENARIO.py:")
    # command = "python3 Onto2OpenSCENARIO.py " + filename_ss
    # print(command)
    # os.system("python3 Onto2OpenSCENARIO.py " + filename_ss)

    # #Bicycle
#    filename_ss = f'anomalous_scenario{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
#    BicycleOnOneWheelOntology("indiv","_",filename_ss)
#    print("Executing Onto2OpenSCENARIO.py:")
#    command = "python3 Onto2OpenSCENARIO.py " + filename_ss
#    print(command)
    # os.system("python3 ConverterV04.py " + filename_ss)

    #running into car
#    filename_ss = f'risky_scenario_running_into_car{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
#    runningIntoCar("indiv","_",filename_ss)
#    print("Executing Onto2OpenSCENARIO.py:")
#    # os.system("python3 /fzi/ids/xt274/no_backup/phd/generation/carla_test2.py" + " & python3 Onto2OpenSCENARIO.py " + filename_ss)
#    os.system("python3 Onto2OpenSCENARIO.py " + filename_ss)


    #merge
#    nameoffile = f'merge_{current_time.tm_hour}_{current_time.tm_min}_{current_time.tm_sec}.owl'
#    test = testMerge("_test","_",nameoffile)
    # os.system("python3 Onto2OpenSCENARIO.py " + nameoffile)
if __name__ == "__main__":
        main()
        print("done")
