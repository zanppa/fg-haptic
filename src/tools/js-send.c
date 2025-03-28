// Tool to send joystick info to flight gear via UDP
#include <stdlib.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_net.h>

#include <stdio.h>		/* printf */
#include <string.h>		/* strstr */
#include <ctype.h>		/* isdigit */

#include <stdbool.h>		/* bool */
#include <math.h>

#include <signal.h>

#include <time.h>

/* From fgfsclient */
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <stdarg.h>

/* #define DFLTHOST        "localhost" */
#define DFLTPORT        5405
#define DFLTHOST		"192.168.56.1"
#define MAX_MSG          512

#define MAX_AXIS	4

// Socket used to communicate with flightgear
UDPsocket server_sock;
bool quit = false;

void abort_execution(int signal);

float clamp(float x, float l, float h)
{
	return ((x) > (h) ? (h) : ((x) < (l) ? (l) : (x)));
}

static UDPpacket *fg_packet = NULL;
void write_fg_generic(void)
{
	const char *p;

	if(!fg_packet)

	//p = fgfsread(client_sock, TIMEOUT);
	if(SDLNet_UDP_Recv(server_sock, fg_packet) < 1)
		return;	// No data received

	p = (char *)fg_packet->data;
	if (!p)
		return;		// Null pointer, read failed
}

/**
 * @brief The entry point of this force feedback demo.
 * @param[in] argc Number of arguments.
 * @param[in] argv Array of argc arguments.
 */
int main(int argc, char **argv)
{
	int i = 0;
	char *name = NULL;
	unsigned int runtime = 0;
	unsigned int dt = 0;
	bool quit = false;
	float jval[MAX_AXIS] = {0.0};	// Joystick axis values
	int channel;

	char address[129] = {0};

	SDL_Joystick *joystick;
	SDL_Event event;

	bool swap_elevator = true;
	bool swap_throttle = true;
	bool scale_throttle = true;

#ifdef _WIN32
    signal(SIGINT, abort_execution);
    signal(SIGTERM, abort_execution);
    signal(SIGABRT, abort_execution);
#else
	struct sigaction signal_handler;

	// Handlers for ctrl+c etc quitting methods
	signal_handler.sa_handler = abort_execution;
	sigemptyset(&signal_handler.sa_mask);
	signal_handler.sa_flags = 0;
	sigaction(SIGINT, &signal_handler, NULL);
	sigaction(SIGQUIT, &signal_handler, NULL);
#endif

	printf("js-send version 0.1\n");
	printf("Sned joystick inputs to Flight Gear\n");
	printf("Copyright 2020 Lauri Peltonen, released under GPLv2 or later\n\n");

	strncpy(address, DFLTHOST, 128);

	// Go through parameters
	for(int i=1;i<argc;i++) {
		name = argv[1];
		if ((strcmp(name, "--help") == 0) || (strcmp(name, "-h") == 0)) {
			printf("USAGE: %s [address] [optional parameters]\n"
				   "	address : Network address to connect to\n"
			       "    -h or --help : Show this help\n"
			       "    --swap_throttle : Swap throttle axis\n"
			       "    --no_scale_throttle : Do not re-scale throttle axis\n"
			       "    --swap_elevator : Swap elevator axis\n\n"
			       "UDP port for FlightGear is %d and generic\n", argv[0], DFLTPORT);
			return 0;
		} else if(strcmp(name, "--swap_throttle") == 0) {
			swap_throttle = false;
		} else if(strcmp(name, "--swap_elevator") == 0) {
			swap_elevator = false;
		} else if(strcmp(name, "--no_scale_throttle") == 0) {
			scale_throttle = false;
		} else if(strlen(name) > 0) {	// Assume any non-known parameter is address
			strncpy(address, name, 128);
		}
	}

	// Initialize SDL
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0)
    {
        fprintf(stderr, "Couldn't initialize SDL: %s\n", SDL_GetError());
        abort_execution(-1);
    }

	// Initialize network
	if(SDLNet_Init() < 0) {
		printf("Could not initialize network: %s\n", SDLNet_GetError());
		abort_execution(-1);
	}

	server_sock = SDLNet_UDP_Open(0);
	if(!server_sock) {
		printf("Could not open UDP port: %s\n", SDLNet_GetError());
		abort_execution(-1);
	}

	IPaddress addr;
	if(SDLNet_ResolveHost(&addr, address, DFLTPORT) < 0) {
		printf("Could not resolve host \"%s\": %s\n", address, SDLNet_GetError());
		abort_execution(-1);
	}
	channel = SDLNet_UDP_Bind(server_sock, -1, &addr);
	if(channel < 0) {
		printf("Could not bind channel: %s\n", SDLNet_GetError());
		abort_execution(-1);
	}

	fg_packet = SDLNet_AllocPacket(MAX_MSG);
	if(!fg_packet) {
		printf("Could not allocate memory for packet\n");
		abort_execution(-1);
	}

    printf("%i joysticks were found.\n\n", SDL_NumJoysticks() );

	if(SDL_NumJoysticks() < 1) {
		abort_execution(-1);
	} else {
	    printf("The names of the joysticks are:\n");
    	for( i=0; i < SDL_NumJoysticks(); i++ )
	    {
    	    printf("    %s\n", SDL_JoystickNameForIndex(i));
	    }
	}

    SDL_JoystickEventState(SDL_ENABLE);
    joystick = SDL_JoystickOpen(0);
	printf("Using joystick %s\n", SDL_JoystickName(joystick));

	// Main loop
	while(!quit)
	{
		dt = runtime;
		runtime = SDL_GetTicks();	// Run time in ms
		dt = runtime - dt;

		while (SDL_PollEvent(&event))		// Loop as long as the connection is alive
		{
			switch(event.type) {
			case SDL_QUIT:
				quit = true;
				abort_execution(0);
				break;

			case SDL_JOYAXISMOTION:
				printf("Axis %d Value %d\n", event.jaxis.axis, event.jaxis.value);
				if(event.jaxis.axis < MAX_AXIS)
					jval[event.jaxis.axis] = event.jaxis.value / 32768.0;
				break;

			case SDL_JOYBUTTONDOWN:
				// event.jbutton.button == 0 ...
				break;

			default:
				break;
			};
		}

		// Send data
		float throttle = swap_throttle ? -jval[2] : jval[2];
		if(scale_throttle)	// Scale from -1 ... 1 to 0...1
			throttle = (throttle + 1.0) * 0.5;

		fg_packet->len = snprintf((char *)fg_packet->data, MAX_MSG-1, "%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f\r\n",
			jval[0],								// Aileron
			swap_elevator ? -jval[1] : jval[1],		// Elevator
			throttle,		// Throttle, engine 1
			throttle,		// Throttle, engine 2
			throttle,		// Throttle, engine 3
			throttle,		// Throttle, engine 4
			jval[3]);		// Rudder
		fg_packet->channel = channel;
		SDLNet_UDP_Send(server_sock, channel, fg_packet);

		SDL_Delay(10);
	}

	SDLNet_UDP_Close(server_sock);
	SDL_Quit();

	return 0;
}

/*
 * Cleans up a bit.
 */
void abort_execution(int signal)
{
	printf("\nAborting program execution.\n");

	if(!fg_packet)
		SDLNet_FreePacket(fg_packet);

	SDLNet_UDP_Close(server_sock);

	SDLNet_Quit();

	SDL_Quit();

	exit(1);
}

