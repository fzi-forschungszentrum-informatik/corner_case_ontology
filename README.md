# Ontology-based corner case scenario simulation for autonomous driving

This repository provides the code for the Bachelor thesis [Ontology-based corner case scenario simulation for autonomous driving](https://publikationen.bibliothek.kit.edu/1000144811) by Stefani Guneshka, supervised by Daniel Bogdoll. Here, an approach for the generation of corner case scenarios within CARLA based on ontologies was created. With this repository you will be able to describe a scenario for autonomous driving, using the provided template ontology, also called master ontology. Once a scenario is described, it can be turned into an OpenSCENARIO file with the help of the Onto2OpenSCENARIOConverter.py, which can be directly executed within [CARLA ScenarioRunner](https://carla-scenariorunner.readthedocs.io/en/latest/). 

## Structure:
```diff
.
├── README.md
├── Steps_add_animals # Steps for adding new assets within CARLA (especially animals, that behave as pedestrians)
├── generation # The main folder that keeps all the information 
│  ├── Onto2OpenSCENARIO.py # The main Converter File, which converts an ontology to an OpenSCENARIO file
│  ├── OntologyGenerator.py # Python script that reads the TemplateOntology.owl and creates new ontology with the same structure and concrete individuals within it
│  ├── ScenarioExamples.py # Example usage of the OntologyGenerator script, on the base of example scenarios
│  ├── TemplateOntology.owl # The main template ontology
│  ├── Videos # Folder with the resulted simulations until now
│  ├── __pycache__
│  ├── carla_get_data.py # Python script, which can be used in combination while scenario_runner is running, to read the information that the ego_vehicle gets.
│  ├── newScenarios # The folder where all new OpenSCENARIO files are stored
│  └── scenarios_new # Folder with old OpenSCENARIO files.
└── old # Folder with old versions of the scripts
  ├── ConverterV04.py 
  ├── MainOntologyV05.owl
  ├── OntologyGeneratorV03.py
  └── carla_get_data.py
```

## How does it work?

The repository consists of 3 main files: TemplateOntology.rdf, OntologyGenerator.py, and Onto2OpenSCENARIOConverter.py. The approach works as follows: The TemplateOntology is used by the OntologyGenerator as a sceleton ontology for all scenario descriptions you want to create. With the help of the OntologyGenerator, one can create and describe a new ontology, which will have the concrete individuals needed for the description of the scenario. Afterwards, the newly created ontology can be read by the Onto2OpenSCENARIOConverter in order to generate the OpenSCENARIO file, needed for the simulation inside CARLA ScenarioRunner. Alternatively, you can use the file and import it into another simulation environment.

The following videos demonstrate scenarios which were created with the concepts, ontology, and tools provided. Further videos can be also seen in the Videos folder.

https://user-images.githubusercontent.com/36483060/163541768-0b812798-b0a5-41b5-8c7c-c8029d3794e5.mp4

https://user-images.githubusercontent.com/36483060/163541822-79ddcf5a-2ab9-4288-9a52-c6c1e055d22a.mp4

## How to contribute
If you want to contribute or add more of the OpenSCENARIO documentation within the repository you need to follow the following steps: 
### 1. Update the TemplateOntology
Here, you can use [Protégé](https://protege.stanford.edu/). This means that you have to create the corresponding classes and also the needed object/data properties.
### 2. Update the OntologyGenerator.
Using the scheme that was used until now for the other classes and parts of OpenSCENARIO, one can improve the OntologyGenerator as well. This would function the following way: 

``` 
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
```
The new individual within the ontology is created by simply using the class name. Afterwards, the name of the new individual follows in the parenthesies. Then, the properties for the individual must be set, and since all properties are arrays, one should use "append". In the end the newly created individual is returned, since it is needed in further functions afterwards.

### 3. Update the Onto2OpenSCENARIO.py. 
This script was created with the help of the [scenariogeneration repository](https://github.com/pyoscx/scenariogeneration). When updating the Onto2OpenSCENARIO file, you need to strictly work with this documentation. Once you know what is required by scenariogeneration, you can read out the information from the ontology and for example compare the class that the individual has, its property assertions or the name of the individual. For the class and names you should always use the `getNameFromIRI()` function, since it returns the real name of the indvidual/class. It is important that you always put ".iri" next to the individual in order to get the IRI of it. 

Note: The function ".is_a[0]" will return the class of the given ontology individual. Example: `getNameFromIRI(entity.is_a[0].iri) == "Vehicle"` will return `True`, if the entity has the class "Vehicle".

## Important notes
In general, in order for the approach to work, you have to install the following [scenariogeneration library](https://github.com/pyoscx/scenariogeneration). You can also use `pip3 install scenariogeneration` to install it. 

Furthermore, when creating a scenario, you can take a look at the "ScenarioExamples.py" file to see how the scenarios until now were created. You should also always load the template ontology within your new python file in order to use its constants and default individuals.
This happens with `template_ontology = get_ontology('TemplateOntology.owl').load())`

Notes: When creating new scenarios with the ontology, be careful to give different names to your new individuals, using the "n1" and "n2" parameters. If two variables have the same name in the ontology, it might lead to an unexpected behaviour. As a tip, if the scenario is not working as expected, check also the names of your new individuals. 

In order  for your scenario runner to work, you need to change the directory path in the "Onto2OpenSCENARIO.py" file to the path of your scenario_runner

## Import new Assets in CARLA

In general, we tried to add new assets within CARLA so that they can be used when creating new scenarios. This was for technical reasons a problem and during the course of the thesis, it was not achieved. However, here I want to provide you with the steps that should allow this to happen.
First you need to [install Unreal Engine 4.26 or higher by forking their repository](https://carla.readthedocs.io/en/latest/build_linux/).
Once this is done and you can import the CARLA project without problems, you can follow the following steps: 

1. You should be able to spawn whatever assets you want, if you first put them into the UnrealEngine "PropFactory": 
![image](https://user-images.githubusercontent.com/36483060/163963204-a94b4d63-ef95-4c1d-8f0c-90b49f5908ec.png)
2. Once you have this opened, you should be able to add a new prop there inside the "PropFactory": 
![image](https://user-images.githubusercontent.com/36483060/163963441-25988b69-236c-4ef2-bb3f-7d03b85286ee.png)
3. After you have added it there, it is importnat to modify the DefaultPackage.json in the CARLA folders, so that the asset is being detected:
![image](https://user-images.githubusercontent.com/36483060/163964135-6dacb509-20d7-457b-876e-17646eeed7d1.png)

This was one of the approaches that we followed, but since we came to problems with installing and opening the project within UnrealEngine, this was not possible. 
Another approach was also proposed, but was conducted on Windows, which caused problems within our Linux build. However, the approach was documented and can be seen in the [Steps_add_animals](https://github.com/daniel-bogdoll/corner_case_ontology/blob/main/Steps_add_animals.pdf) file in this repository. This method was used to specially add animals, which behave like a pedestrian inside CARLA.

## Citation
If you find this code useful for your research, please cite our paper:
```
@mastersthesis{Guneshka_Ontology_2022_BA,
  author    = {Guneshka, Stefani},
  title     = {{Ontology-based Corner Case Scenario Simulation for Autonomous Driving}},
  school    = {{Karlsruhe Institute of Technology (KIT)}},
  year      = {2022},
  type      = {Bachelor Thesis}
}
```
