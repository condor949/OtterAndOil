###############################################################################
# Function printVehicleinfo(vehicle)
###############################################################################
def printVehicleinfo(vehicle):
    print(f'---{vehicle}--------------------------------------------------------------------------')
    print(f'{vehicle.name}')
    print(f'Length: {vehicle.L} m')
    print(f'Control: {vehicle.controlDescription}')
    print(f'Starting point: [{vehicle.starting_point[0]}, {vehicle.starting_point[1]}]')


def printControllerinfo(controller):
    print(f'---{controller}--------------------------------------------------------------------------')
    print(f'Sampling frequency: {round(1 / controller.sample_time)} Hz')
    print(f'Simulation time: {round(controller.sim_time)} seconds')


def printSpaceinfo(space):
    print(f'---{space}--------------------------------------------------------------------------')
    print(f'{space.name}')
    print(f'Shifting space: {space.shift_xyz} m')
