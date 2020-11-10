# driver/scripts
## Files:
- logicModels.py
    - sub to:
        - /gazebo/get_field
    - pub to:
        - /field/models
        - /field/zones
        - /field/types
    - purpose:
        - publish data on the models, zones, and types for processing
        - assign models to zones
        - sort zones to display correctly
        - acts as the config file

- costModels.py
    - sub to:
        - /field/zones
        - /field/types
    - pub to:
        - /gazebo/set_field

- displayModels.py
    - sub to:
        - /field/zones
        - /field/types
    - pub to:
        - N/A
    - purpose:
        - get data from zones to draw
        - get data from types on how to draw
        - a real time field animation from the birds eye

- fakeGazebo.py
    - sub to:
        - /gazebo/set_field
        - /gazebo/get_field
    - pub to:
        - /field/models
        - /gazebo/fake
    - purpose:
        - fake gazebo by passing the data from /gazebo/set_field to /field/models
        - make /gazebo/fake True
 