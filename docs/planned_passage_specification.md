# Following a planned passage exeriment

The goal is to follow a planned passage. This planned passage will be defined using file defining the route and the expected speed and time between each course change, and wind. In addition the gibs containing wind speed and sea state will be made aviable. All of this defines the predicted route and arrival time having optimised the route for performance.


The test is for the autopilot model to follow the route using the polar with wind gribs to predict the speed through the water, and the wave gribs to simulate the sea state. 

The route information will indicate if the boat will be motoring or sailing. AWA less than 130 should steer using AWA and TWA > 130 should use TWA to steer to.

All code required to implement this should in put into a seperate package experiments/experiment1 to keep it seperate from other code.

The gribs for this expiment can be found in data/experiment1/gribs/ they contain a wide range of parameters covering 2 days and cover the sea area in 2 resolutions.

The calculated route to sail can be found in both kml and csv format in data/experiment1/gribs