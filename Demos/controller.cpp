#include <TelloDrone.h>
#include <iostream>
#include <SDL.h>
#include <SDL_gamecontroller.h>

static constexpr auto dead_zone = 3000;

int main()
{
    if (SDL_WasInit(SDL_INIT_GAMECONTROLLER) != 1)
        SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER);

    auto number_of_joysticks = SDL_NumJoysticks();
    auto game_controller_index = -1;
    for (auto i = 0; i < number_of_joysticks; ++i) {
        if (SDL_IsGameController(i)) {
            game_controller_index = i;
            break;
        }
    }

    if (game_controller_index == -1) {
        std::cerr << "No game controller was found!" << std::endl;
        return 1;
    }
    std::cout << "Attaching to " << SDL_JoystickNameForIndex(game_controller_index) << std::endl;

    auto* game_controller = SDL_GameControllerOpen(game_controller_index);
    if (SDL_GameControllerGetAttached(game_controller) != 1) {
        std::cerr << "Unable to attach to game controller! SDL Error: " << SDL_GetError() << std::endl;
        return 1;
    }
    auto controller_instance_id = SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(game_controller));

    Tello::Drone drone;
    std::cout << "Connecting to the drone..." << std::endl;
    drone.wait_until_connected();
    std::cout << "Connected to the drone!" << std::endl;

    SDL_GameControllerEventState(SDL_ENABLE);
    i16 axes[SDL_CONTROLLER_AXIS_MAX] = { 0 };
    i16 offset[SDL_CONTROLLER_AXIS_MAX] = { 0 };
    auto sdl_axis_to_drone = [&axes, &offset](auto axis) {
        auto axis_value = axes[axis] - offset[axis];
        if (abs(axis_value) <= dead_zone)
            return 0.0f;
        return (float)(axis_value) / 32768.0f;
    };
    while(true) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            switch(event.type) {
            case SDL_QUIT: {
                return 0;
            }
            case SDL_CONTROLLERBUTTONDOWN: {
                if (event.cbutton.which != controller_instance_id)
                    break;
                if (event.cbutton.button == SDL_CONTROLLER_BUTTON_Y) {
                    std::cout << "Taking off..." << std::endl;
                    drone.take_off_non_blocking();
                } else if (event.cbutton.button == SDL_CONTROLLER_BUTTON_A) {
                    std::cout << "Landing..." << std::endl;
                    drone.land_non_blocking();
                } else if (event.cbutton.button == SDL_CONTROLLER_BUTTON_B) {
                    for(auto i = 0; i < SDL_CONTROLLER_AXIS_MAX; ++i)
                        offset[i] = axes[i];
                    std::cout << "Saved re-centering offset!" << std::endl;
                }
                break;
            }
            case SDL_CONTROLLERAXISMOTION: {
                if (event.caxis.which != controller_instance_id)
                    break;
                axes[event.caxis.axis] = event.caxis.value;
                drone.set_joysticks_state(sdl_axis_to_drone(SDL_CONTROLLER_AXIS_RIGHTX), -sdl_axis_to_drone(SDL_CONTROLLER_AXIS_RIGHTY), sdl_axis_to_drone(SDL_CONTROLLER_AXIS_LEFTX), -sdl_axis_to_drone(SDL_CONTROLLER_AXIS_LEFTY));
                break;
            }
            }
        }
    }
}
