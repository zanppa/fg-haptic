// Haptic
#include <stdlib.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_haptic.h>
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
#define DFLTPORT        5401
#define MAXMSG          512
#define fgfsclose       SDLNet_TCP_Close

#define NAMELEN		30

#define READ_TIMEOUT	30	// 30 secs, timeout when reading telnet data (should be long)
#define CONN_TIMEOUT	30	// 30 seconds, timeout when connecting the telnet connection
#define DISCONNECT_TIMEOUT	5	// After 5 seconds of no data (after valid data) consider fgfs closed (ms)
#define AXES		3	// Maximum axes supported

#define CONST_X		0
#define CONST_Y		1
#define CONST_Z		2
#define SPRING		3
#define STICK_SHAKER	4
#define GROUND_NOTCH	5
#define FRICTION	6
#define DAMPING		7

#define EFFECTS		9	// Maximum effects


#define MODE_OFF	-1
#define MODE_NORMAL	0
#define MODE_ALTERNATE	1
const char axes[AXES] = { 'x', 'y', 'z' };

//void init_sockaddr(struct sockaddr_in *name, const char *hostname, unsigned port);
TCPsocket fgfsconnect(const Uint32 host, const Uint16 port);
UDPsocket fgfs_udp_reader(const Uint16 port);
Uint32 fgfs_wait_packet(UDPsocket socket);
int fgfswrite(TCPsocket sock, char *msg, ...);
const char *fgfsread(TCPsocket sock, int wait);
void fgfsflush(TCPsocket sock);

// Socket used to communicate with flightgear
TCPsocket telnet_sock;
UDPsocket server_sock;
IPaddress *client_addr;
SDLNet_SocketSet socketset;
static UDPpacket *fg_packet = NULL;

// Effect struct definitions, used to store parameters
typedef struct __effectParams {
	float pilot[AXES];
	float stick[AXES];
	float trim[AXES];
	int shaker_trigger;

	float rumble_period;	// Ground rumble period, 0=disable

	union  {
		struct {
			float x;		// Forces (pilot)
			float y;
			float z;
		} d;
		float f[3];
	} force;

	float coeff[AXES];	// Spring effect coefficients
} effectParams;

int num_devices;

typedef struct __hapticdevice {
	SDL_Haptic *device;
	char name[NAMELEN + 1];	// Name
	unsigned int num;	// Num of this device
	unsigned int supported;	// Capabilities
	unsigned int axes;	// Count of axes
	unsigned int numEffects, numEffectsPlaying;
	bool open;

	SDL_HapticEffect effect[EFFECTS];
	int effectId[EFFECTS];

	effectParams params;

	// Configuration
	float autocenter;
	float gain;

	unsigned short shaker_dir;
	unsigned short shaker_period;

	float pilot_gain;
	float stick_gain;
	float shaker_gain;
	float rumble_gain;
	float friction;
	float damping;

	signed char rumble_mode;
	signed char stick_mode;

	signed char pilot_axes[AXES];	// Axes mapping, -1 = not used. Pilot axis to joystick axis
	signed char stick_axes[AXES];

	signed char pilot_invert[AXES];	// 1 = non-inverted, -1 = inverted
	signed char stick_invert[AXES];

	unsigned int last_rumble;

	float lowpass;		// Low pass filter tau, in ms

} hapticDevice;

static hapticDevice *devices = NULL;
int reconf_request = false;
bool quit = false;

effectParams new_params;

/*
 * prototypes
 */
void abort_execution(int signal);
void HapticPrintSupported(SDL_Haptic * haptic);

float clamp(float x, float l, float h)
{
	return ((x) > (h) ? (h) : ((x) < (l) ? (l) : (x)));
}

void init_haptic(void)
{
	/* Initialize the force feedbackness */
	SDL_Init(SDL_INIT_TIMER | SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC);

	num_devices = SDL_NumHaptics();
	printf("%d Haptic devices detected.\n", num_devices);

	devices = (hapticDevice *) malloc(num_devices * sizeof(hapticDevice));
	if (!devices) {
		printf("Fatal error: Could not allocate memory for devices!\n");
		abort_execution(-1);
	}
	// Zero
	memset(devices, 0, num_devices * sizeof(hapticDevice));

	// Parse device data
	for (int i = 0; i < num_devices; i++) {
		devices[i].num = i + 1;	// Add one, so we get around flightgear reading empty properties as 0
		devices[i].device = SDL_HapticOpen(i);

		if (devices[i].device) {
			devices[i].open = true;

			HapticPrintSupported(devices[i].device);
			// Copy devices name with ascii
			const char *p = SDL_HapticName(i);
			strncpy(devices[i].name, p, NAMELEN);

			// Add device number after name, if there is multiples with same name
			for (int a = 0; a < i; a++) {
				if (strcmp(devices[i].name, devices[a].name) == 0) {
					size_t len = strlen(devices[i].name);
					if (len < NAMELEN - 2) {	// Enough space to add number after name
						devices[i].name[len] = ' ';
						devices[i].name[len + 1] = '1' + i;
					} else {
						devices[i].name[NAMELEN - 2] = ' ';
						devices[i].name[NAMELEN - 1] = '1' + i;
					}
				}
			}

			printf("Device %d name is %s\n", devices[i].num, devices[i].name);

			// Capabilities
			devices[i].supported = SDL_HapticQuery(devices[i].device);
			devices[i].axes = SDL_HapticNumAxes(devices[i].device);
			if (devices[i].axes > AXES)
				devices[i].axes = AXES;
			devices[i].numEffects = SDL_HapticNumEffects(devices[i].device);
			devices[i].numEffectsPlaying = SDL_HapticNumEffectsPlaying(devices[i].device);

			// Default effect parameters
			// By default map aileron to X, elevator to Y and rudder to Z
			for (int a = 0; a < devices[i].axes; a++) {
				devices[i].stick_axes[a] = a < AXES ? a : -1;	// Only map axis that exist in the joystick
				devices[i].stick_invert[a] = 1;
				devices[i].pilot_invert[a] = 1;
			}

			// By default only Z axis is mapped to joystick Y in pilot forces
			devices[i].pilot_axes[0] = -1;	// Pilot X (sideways) to nowhere
			devices[i].pilot_axes[1] = -1;	// Pilot Y (front to back) to nowhere
			devices[i].pilot_axes[2] = 1;	// Pilot Z (up and down) to joystick Y

			devices[i].autocenter = 0.0;
			devices[i].gain = 1.0;
			devices[i].pilot_gain = 0.3;
			devices[i].stick_gain = 1.0;
			devices[i].stick_mode = MODE_NORMAL;
			devices[i].shaker_gain = 0.4;
			devices[i].shaker_period = 100.0;
			devices[i].rumble_gain = 0.25;
			devices[i].rumble_mode = MODE_NORMAL;
			devices[i].lowpass = 50.0;

		} else {
			printf("Unable to open haptic devices %d: %s\n", i, SDL_GetError());
			devices[i].open = false;
		}
	}

	/* We only want force feedback errors. */
	SDL_ClearError();
}

void send_devices(void)
{
	// Init general properties
	fgfswrite(telnet_sock, "set /haptic/reconfigure 0");

	// Init devices
	for (int i = 0; i < num_devices; i++) {
		// Write devices to flightgear
		fgfswrite(telnet_sock, "set /haptic/device[%d]/number %d", i, devices[i].num);
		fgfswrite(telnet_sock, "set /haptic/device[%d]/name %s", i, devices[i].name);
		fgfswrite(telnet_sock, "set /haptic/device[%d]/supported %d", i, devices[i].supported);
		fgfswrite(telnet_sock, "set /haptic/device[%d]/axes %d", i, devices[i].axes);
		fgfswrite(telnet_sock, "set /haptic/device[%d]/num-effects %d", i, devices[i].numEffects);
		fgfswrite(telnet_sock, "set /haptic/device[%d]/num-effects-playing %d", i, devices[i].numEffectsPlaying);

		fgfswrite(telnet_sock, "set /haptic/device[%d]/low-pass-filter %.6f", i, devices[i].lowpass);

		// Write supported effects
		if (devices[i].supported & SDL_HAPTIC_CONSTANT) {
			// Constant force -> pilot G forces and aileron loading
			// Currently support 3 axis only
			for (int x = 0; x < AXES; x++) {
				fgfswrite(telnet_sock, "set /haptic/device[%d]/pilot/%c %d", i, axes[x], devices[i].pilot_axes[x] + (devices[i].pilot_invert[x] < 0 ? AXES : 0));
				fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-force/%c %d", i, axes[x], devices[i].stick_axes[x] + (devices[i].stick_invert[x] < 0 ? AXES : 0));
			}
			fgfswrite(telnet_sock, "set /haptic/device[%d]/pilot/gain %f", i, devices[i].pilot_gain);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/pilot/supported 1", i);

			fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-force/supported 1", i);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-force/alternate 1", i);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-force/gain %f", i, devices[i].stick_gain);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-force/mode 1", i);

			fgfswrite(telnet_sock, "set /haptic/device[%d]/ground-rumble/supported 1", i);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/ground-rumble/alternate 1", i);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/ground-rumble/period 0.0", i);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/ground-rumble/gain %f", i, devices[i].rumble_gain);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/ground-rumble/mode 0", i);
		}

		if (devices[i].supported & SDL_HAPTIC_SPRING) {
			fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-force/supported 1", i);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-force/normal 1", i);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-force/gain %f", i, devices[i].stick_gain);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-force/mode 0", i);
		}

		if (devices[i].supported & SDL_HAPTIC_SINE) {
			// Sine effect -> rumble is stick shaker
			fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-shaker/supported 1", i);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-shaker/direction %f", i, devices[i].shaker_dir);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-shaker/period %f", i, devices[i].shaker_period);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-shaker/gain %f", i, devices[i].shaker_gain);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/stick-shaker/trigger 0", i);
		}

		if (devices[i].supported & SDL_HAPTIC_SAWTOOTHDOWN) {
			fgfswrite(telnet_sock, "set /haptic/device[%d]/ground-rumble/supported 1", i);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/ground-rumble/normal 1", i);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/ground-rumble/period 0.0", i);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/ground-rumble/gain %f", i, devices[i].rumble_gain);
		}

		if (devices[i].supported & SDL_HAPTIC_GAIN) {
			fgfswrite(telnet_sock, "set /haptic/device[%d]/gain %f", i, devices[i].gain);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/gain-supported 1", i);
		}
		if (devices[i].supported & SDL_HAPTIC_AUTOCENTER) {
			fgfswrite(telnet_sock, "set /haptic/device[%d]/autocenter %f", i, devices[i].autocenter);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/autocenter-supported 1", i);
		}
		if (devices[i].supported & SDL_HAPTIC_FRICTION) {
			fgfswrite(telnet_sock, "set /haptic/device[%d]/friction %f", i, devices[i].friction);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/friction-supported 1", i);
		}
		if (devices[i].supported & SDL_HAPTIC_DAMPER) {
			fgfswrite(telnet_sock, "set /haptic/device[%d]/damping %f", i, devices[i].damping);
			fgfswrite(telnet_sock, "set /haptic/device[%d]/damping-supported 1", i);
		}
	}

	// Tell fgfs that we have reconfigured the property tree for haptic devices
	fgfswrite(telnet_sock, "set /haptic/devices-reconfigured 1");
}

void read_devices(void)
{
	int idata;
	float fdata;
	int read = 0;
	const char *p;

	fgfsflush(telnet_sock);

	printf("Reading device setup from FG\n");

	for (int i = 0; i < num_devices; i++) {
		printf("  Device %d\n", i);
		// Constant device settings
		fgfswrite(telnet_sock, "get /haptic/device[%d]/low-pass-filter", i);
		p = fgfsread(telnet_sock, READ_TIMEOUT);
		if (p) {
			read = sscanf(p, "%f", &fdata);
			if (read == 1) {
				devices[i].lowpass = fdata;
				printf("    LP filter %f\n", fdata);
			}
		}

		if (devices[i].supported & SDL_HAPTIC_GAIN) {
			fgfswrite(telnet_sock, "get /haptic/device[%d]/gain", i);
			p = fgfsread(telnet_sock, READ_TIMEOUT);
			if (p) {
				read = sscanf(p, "%f", &fdata);
				if (read == 1) {
					devices[i].gain = fdata;
					printf("    Gain %f\n", fdata);
				}
			}
		}

		if (devices[i].supported & SDL_HAPTIC_AUTOCENTER) {
			fgfswrite(telnet_sock, "get /haptic/device[%d]/autocenter", i);
			p = fgfsread(telnet_sock, READ_TIMEOUT);
			if (p) {
				read = sscanf(p, "%f", &fdata);
				if (read == 1) {
					devices[i].autocenter = fdata;
					printf("    Autocenter %f\n", fdata);
				}
			}
		}

		// Constant force -> pilot G forces and aileron loading
		// Currently support 3 axis only
		if (devices[i].supported & SDL_HAPTIC_CONSTANT) {
			fgfswrite(telnet_sock, "get /haptic/device[%d]/pilot/gain", i);
			p = fgfsread(telnet_sock, READ_TIMEOUT);
			if (p) {
				read = sscanf(p, "%f", &fdata);
				if (read == 1) {
					devices[i].pilot_gain = fdata;
					printf("    Pilot gain %f\n", fdata);
				}
			}
			fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-force/gain", i);
			p = fgfsread(telnet_sock, READ_TIMEOUT);
			if (p) {
				read = sscanf(p, "%f", &fdata);
				if (read == 1) {
					devices[i].stick_gain = fdata;
					printf("    Stick force gain %f\n", fdata);
				}
			}
			fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-force/mode", i);
			p = fgfsread(telnet_sock, READ_TIMEOUT);
			if (p) {
				read = sscanf(p, "%d", &idata);
				if (read == 1) {
					devices[i].stick_mode = idata;
					printf("    Stick force mode %d\n", idata);
				}
			}

			fgfswrite(telnet_sock, "get /haptic/device[%d]/ground-rumble/gain", i);
			p = fgfsread(telnet_sock, READ_TIMEOUT);
			if (p) {
				read = sscanf(p, "%f", &fdata);
				if (read == 1) {
					devices[i].rumble_gain = fdata;
					printf("    Ground rumble gain %f\n", fdata);
				}
			}
			fgfswrite(telnet_sock, "get /haptic/device[%d]/ground-rumble/mode", i);
			p = fgfsread(telnet_sock, READ_TIMEOUT);
			if (p) {
				read = sscanf(p, "%d", &idata);
				if (read == 1) {
					devices[i].rumble_mode = idata;
					printf("    Ground rumble mode %d\n", idata);
				}
			}

			for (int x = 0; x < AXES; x++) {
				fgfswrite(telnet_sock, "get /haptic/device[%d]/pilot/%c", i, axes[x]);
				p = fgfsread(telnet_sock, READ_TIMEOUT);
				if (p) {
					read = sscanf(p, "%d", &idata);
					if (read == 1) {
						if(idata >= AXES) {
							devices[i].pilot_invert[x] = -1;
							idata = idata % AXES;
						} else devices[i].pilot_invert[x] = 1;

						devices[i].pilot_axes[x] = idata;
					}
					printf("Pilot force axis (sim) %c: (joystick) %c %s\n", axes[x], axes[devices[i].pilot_axes[x]], devices[i].pilot_invert[x]>0?"":"inv");
				}

				fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-force/%c", i, axes[x]);
				p = fgfsread(telnet_sock, READ_TIMEOUT);
				if (p) {
					read = sscanf(p, "%d", &idata);
					if (read == 1) {
						if(idata >= AXES) {
							devices[i].stick_invert[x] = -1;
							idata = idata % AXES;
						} else devices[i].stick_invert[x] = 1;
						devices[i].stick_axes[x] = idata;
					}
					printf("Stick force axis (sim) %c: (joystick) %c %s\n", axes[x], axes[devices[i].stick_axes[x]], devices[i].stick_invert[x]>0?"":"inv");
				}
			}
		}

		if (devices[i].supported & SDL_HAPTIC_SINE) {
			fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-shaker/direction", i);
			p = fgfsread(telnet_sock, READ_TIMEOUT);
			if (p) {
				read = sscanf(p, "%f", &fdata);
				if (read == 1)
					devices[i].shaker_dir = fdata;
			}
			fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-shaker/period", i);
			p = fgfsread(telnet_sock, READ_TIMEOUT);
			if (p) {
				read = sscanf(p, "%f", &fdata);
				if (read == 1)
					devices[i].shaker_period = fdata;
			}
			fgfswrite(telnet_sock, "get /haptic/device[%d]/stick-shaker/gain", i);
			p = fgfsread(telnet_sock, READ_TIMEOUT);
			if (p) {
				read = sscanf(p, "%f", &fdata);
				if (read == 1)
					devices[i].shaker_gain = fdata;
			}
		}

		if (devices[i].supported & SDL_HAPTIC_FRICTION) {
			fgfswrite(telnet_sock, "get /haptic/device[%d]/friction", i);
			p = fgfsread(telnet_sock, READ_TIMEOUT);
			if (p) {
				read = sscanf(p, "%f", &fdata);
				if (read == 1) {
					devices[i].friction = fdata;
					printf("    Friction %f\n", fdata);
				}
			}
		}

		if (devices[i].supported & SDL_HAPTIC_DAMPER) {
			fgfswrite(telnet_sock, "get /haptic/device[%d]/damping", i);
			p = fgfsread(telnet_sock, READ_TIMEOUT);
			if (p) {
				read = sscanf(p, "%f", &fdata);
				if (read == 1) {
					devices[i].damping = fdata;
					printf("    Damping %f\n", fdata);
				}
			}
		}

	}

	int retries = 0;
	idata = 1;
	do {
		fgfswrite(telnet_sock, "set /haptic/reconfigure 0");
		printf("Marking reconfiguration done...\n");
		int reads = 0;
		do {
			reads++;
			idata = 1;
			fgfswrite(telnet_sock, "get /haptic/reconfigure");
			p = fgfsread(telnet_sock, READ_TIMEOUT);
			if (p)
				read = sscanf(p, "%d", &idata);
			else
				read = 0;
		} while(idata && reads < 30);
		retries++;

		if(!idata) break;

		SDL_Delay(5000);	// Wait 5 seconds before trying again
	} while(retries < 20);
	printf("Done\n");

	//fgfsflush(client_sock);	// Get rid of FF data that was received during reinitialization

	// Flush UDP
	if(fg_packet)
		while(SDLNet_UDP_Recv(server_sock, fg_packet) > 0);

	return;
}

void create_effects(void)
{
	for (int i = 0; i < num_devices; i++) {
		// Delete existing effects
		for (int x = 0; x < SDL_HapticNumEffects(devices[i].device); x++) {
			SDL_HapticDestroyEffect(devices[i].device, x);
			if (x < sizeof(devices[i].effectId) / sizeof(devices[i].effectId[0])) {
				devices[i].effectId[x] = -1;
			}
		}

		memset(&devices[i].effect[0], 0, sizeof(SDL_HapticEffect) * EFFECTS);

		printf("Creating effects for device %d\n", i + 1);

		// Set autocenter and gain
		if (devices[i].supported & SDL_HAPTIC_AUTOCENTER)
			SDL_HapticSetAutocenter(devices[i].device, devices[i].autocenter * 100);

		if (devices[i].supported & SDL_HAPTIC_GAIN)
			SDL_HapticSetGain(devices[i].device, devices[i].gain * 100);

		// Spring effect for stick forces (normal mode)
		if (devices[i].supported & SDL_HAPTIC_SPRING && devices[i].stick_gain > 0.001 && devices[i].stick_mode == MODE_NORMAL) {
			printf("\tSpring effect\n");
			devices[i].effect[SPRING].type = SDL_HAPTIC_SPRING;
			devices[i].effect[SPRING].condition.length = SDL_HAPTIC_INFINITY;

			for(int j = 0; j < AXES; j++) {
				devices[i].effect[SPRING].condition.left_sat[j] = 0xFFFF;
				devices[i].effect[SPRING].condition.right_sat[j] = 0xFFFF;
				devices[i].effect[SPRING].condition.left_coeff[j] = 0;
				devices[i].effect[SPRING].condition.right_coeff[j] = 0;
				devices[i].effect[SPRING].condition.deadband[j] = 0;
				devices[i].effect[SPRING].condition.center[j] = 0;
			}

			devices[i].effectId[SPRING] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[SPRING]);
			if (devices[i].effectId[SPRING] < 0) {
				printf("UPLOADING EFFECT ERROR: %s\n", SDL_GetError());
				devices[i].effectId[SPRING] = -1;
				//devices[i].supported &= ~SDL_HAPTIC_SPRING;
			}
		}

		// Stick shaker
		if (devices[i].supported & SDL_HAPTIC_SINE && devices[i].shaker_gain > 0.001) {
			printf("\tStick shaker\n");
			devices[i].effect[STICK_SHAKER].type = SDL_HAPTIC_SINE;
			devices[i].effect[STICK_SHAKER].periodic.direction.type = SDL_HAPTIC_POLAR;
			devices[i].effect[STICK_SHAKER].periodic.direction.dir[0] = devices[i].shaker_dir;
			devices[i].effect[STICK_SHAKER].periodic.direction.dir[1] = 0;
			devices[i].effect[STICK_SHAKER].periodic.direction.dir[2] = 0;
			devices[i].effect[STICK_SHAKER].periodic.length = SDL_HAPTIC_INFINITY;
			devices[i].effect[STICK_SHAKER].periodic.period = devices[i].shaker_period;
			devices[i].effect[STICK_SHAKER].periodic.magnitude = 32767 * devices[i].shaker_gain;
			devices[i].effect[STICK_SHAKER].periodic.attack_length = 300;	// 0.3 sec fade in
			devices[i].effect[STICK_SHAKER].periodic.fade_length = 300;	// 0.3 sec fade out

			devices[i].effectId[STICK_SHAKER] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[STICK_SHAKER]);
			if (devices[i].effectId[STICK_SHAKER] < 0) {
				printf("UPLOADING EFFECT ERROR: %s\n", SDL_GetError());
				devices[i].effectId[STICK_SHAKER] = -1;
				//devices[i].supported &= ~SDL_HAPTIC_SINE;
			}
		}

		// Ground (asphalt) rumble effect
		if (devices[i].supported & SDL_HAPTIC_SAWTOOTHDOWN && devices[i].rumble_gain > 0.001) {
			printf("\tGround rumble\n");
			devices[i].effect[GROUND_NOTCH].type = SDL_HAPTIC_SAWTOOTHDOWN;
			devices[i].effect[GROUND_NOTCH].periodic.direction.type = SDL_HAPTIC_POLAR;
			devices[i].effect[GROUND_NOTCH].periodic.direction.dir[0] = 0;
			devices[i].effect[GROUND_NOTCH].periodic.direction.dir[1] = 0;
			devices[i].effect[GROUND_NOTCH].periodic.direction.dir[2] = 0;
			devices[i].effect[GROUND_NOTCH].periodic.length = SDL_HAPTIC_INFINITY;
			devices[i].effect[GROUND_NOTCH].periodic.delay = 0;
			devices[i].effect[GROUND_NOTCH].periodic.period = 1000;	// Temporary placeholder 1 s
			devices[i].effect[GROUND_NOTCH].periodic.magnitude = (int)(devices[i].rumble_gain * 32767);
			devices[i].effect[GROUND_NOTCH].periodic.offset = devices[i].effect[GROUND_NOTCH].periodic.magnitude / 2;
			devices[i].effect[GROUND_NOTCH].periodic.phase = 0;
			devices[i].effect[GROUND_NOTCH].periodic.attack_length = 0;
			devices[i].effect[GROUND_NOTCH].periodic.attack_level = 0;
			devices[i].effect[GROUND_NOTCH].periodic.fade_length = 0;
			devices[i].effect[GROUND_NOTCH].periodic.fade_level = 0;

			devices[i].effectId[GROUND_NOTCH] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[GROUND_NOTCH]);
			if (devices[i].effectId[GROUND_NOTCH] < 0) {
				printf("UPLOADING GROUND RUMBLE EFFECT ERROR: %s\n", SDL_GetError());
				devices[i].effectId[GROUND_NOTCH] = -1;
				//devices[i].supported &= ~SDL_HAPTIC_CONSTANT;
			}
		}

		// X axis
		if (devices[i].supported & SDL_HAPTIC_CONSTANT && devices[i].axes > 0) {
			devices[i].effect[CONST_X].type = SDL_HAPTIC_CONSTANT;
			devices[i].effect[CONST_X].constant.direction.type = SDL_HAPTIC_CARTESIAN;
			devices[i].effect[CONST_X].constant.direction.dir[0] = 1;
			devices[i].effect[CONST_X].constant.direction.dir[1] = 0;
			devices[i].effect[CONST_X].constant.direction.dir[2] = 0;
			devices[i].effect[CONST_X].constant.length = SDL_HAPTIC_INFINITY;
			devices[i].effect[CONST_X].constant.level = 0x7000;

			devices[i].effectId[CONST_X] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[CONST_X]);
			if (devices[i].effectId[CONST_X] < 0) {
				printf("UPLOADING CONST_X EFFECT ERROR: %s\n", SDL_GetError());
				devices[i].effectId[CONST_X] = -1;
				// devices[i].supported &= ~SDL_HAPTIC_CONSTANT;
			}
			printf("\tConstant force X\n");
		}
		// Y axis
		if (devices[i].supported & SDL_HAPTIC_CONSTANT && devices[i].axes > 1) {
			devices[i].effect[CONST_Y].type = SDL_HAPTIC_CONSTANT;
			devices[i].effect[CONST_Y].constant.direction.type = SDL_HAPTIC_CARTESIAN;
			devices[i].effect[CONST_Y].constant.direction.dir[0] = 0;
			devices[i].effect[CONST_Y].constant.direction.dir[1] = -1;
			devices[i].effect[CONST_Y].constant.direction.dir[2] = 0;
			devices[i].effect[CONST_Y].constant.length = SDL_HAPTIC_INFINITY;
			devices[i].effect[CONST_Y].constant.level = 0x7000;

			devices[i].effectId[CONST_Y] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[CONST_Y]);
			if (devices[i].effectId[CONST_Y] < 0) {
				printf("UPLOADING CONST_Y EFFECT ERROR: %s\n", SDL_GetError());
				devices[i].effectId[CONST_Y] = -1;
				// devices[i].supported &= ~SDL_HAPTIC_CONSTANT;
			}
			printf("\tConstant force Y\n");
		}
		// Z axis
		if (devices[i].supported & SDL_HAPTIC_CONSTANT && devices[i].axes > 2) {
			devices[i].effect[CONST_Z].type = SDL_HAPTIC_CONSTANT;
			devices[i].effect[CONST_Z].constant.direction.type = SDL_HAPTIC_CARTESIAN;
			devices[i].effect[CONST_Z].constant.direction.dir[0] = 0;
			devices[i].effect[CONST_Z].constant.direction.dir[1] = 0;
			devices[i].effect[CONST_Z].constant.direction.dir[2] = 1;	// Up?
			devices[i].effect[CONST_Z].constant.length = SDL_HAPTIC_INFINITY;
			devices[i].effect[CONST_Z].constant.level = 0x1000;

			devices[i].effectId[CONST_Z] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[CONST_Z]);
			if (devices[i].effectId[CONST_Z] < 0) {
				printf("UPLOADING CONST_Z EFFECT ERROR: %s\n", SDL_GetError());
				devices[i].effectId[CONST_Z] = -1;
				// devices[i].supported &= ~SDL_HAPTIC_CONSTANT;
			}
			printf("\tConstant force Z\n");
		}

		// Friction effect, force opposing stick movement (static friction), currently constant gain for all axis
		if (devices[i].supported & SDL_HAPTIC_FRICTION && devices[i].friction > 0.001) {
			printf("\tFriction effect\n");
			devices[i].effect[FRICTION].type = SDL_HAPTIC_FRICTION;
			devices[i].effect[FRICTION].condition.length = SDL_HAPTIC_INFINITY;

			for(int j = 0; j < AXES; j++) {
				devices[i].effect[FRICTION].condition.left_sat[j] = 0xFFFF;
				devices[i].effect[FRICTION].condition.right_sat[j] = 0xFFFF;
				devices[i].effect[FRICTION].condition.left_coeff[j] = 32760 * devices[i].friction;
				devices[i].effect[FRICTION].condition.right_coeff[j] = 32760 * devices[i].friction;
				devices[i].effect[FRICTION].condition.deadband[j] = 0;
				devices[i].effect[FRICTION].condition.center[j] = 0;
			}

			devices[i].effectId[FRICTION] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[FRICTION]);
			if (devices[i].effectId[FRICTION] < 0) {
				printf("UPLOADING EFFECT ERROR: %s\n", SDL_GetError());
				devices[i].effectId[FRICTION] = -1;
			}
		}

		// Damping effect, force opposing stick velocity (higher velocity -> more force needed), constant gain in all axis
		if (devices[i].supported & SDL_HAPTIC_DAMPER && devices[i].damping > 0.001) {
			printf("\tDamping effect\n");
			devices[i].effect[DAMPING].type = SDL_HAPTIC_DAMPER;
			devices[i].effect[DAMPING].condition.length = SDL_HAPTIC_INFINITY;

			for(int j = 0; j < AXES; j++) {
				devices[i].effect[DAMPING].condition.left_sat[j] = 0xFFFF;
				devices[i].effect[DAMPING].condition.right_sat[j] = 0xFFFF;
				devices[i].effect[DAMPING].condition.left_coeff[j] = 32760 * devices[i].damping;
				devices[i].effect[DAMPING].condition.right_coeff[j] = 32760 * devices[i].damping;
				devices[i].effect[DAMPING].condition.deadband[j] = 0;
				devices[i].effect[DAMPING].condition.center[j] = 0;
			}

			devices[i].effectId[DAMPING] = SDL_HapticNewEffect(devices[i].device, &devices[i].effect[DAMPING]);
			if (devices[i].effectId[DAMPING] < 0) {
				printf("UPLOADING EFFECT ERROR: %s\n", SDL_GetError());
				devices[i].effectId[DAMPING] = -1;
			}
		}

	}
}

void reload_effect(hapticDevice * device, SDL_HapticEffect * effect, int *effectId, bool run)
{
	if (!device->device || !device->open) {
		printf("Error: device not open!\n");
		return;
	}

	if (SDL_HapticUpdateEffect(device->device, *effectId, effect) < 0)
		printf("Update error: %s\n", SDL_GetError());
	if (run) {
		//printf("Running effect %d\n", *effectId);
		if (SDL_HapticRunEffect(device->device, *effectId, 1) < 0)
			printf("Run error: %s\n", SDL_GetError());
	}
}


// Read the generic UDP protocol
// Return 0 if valid data was received, otherwise negative number
int read_fg_generic(void)
{
	int reconf, read;
	char *p;
	effectParams params;

	if(!fg_packet)
		fg_packet = SDLNet_AllocPacket(1024);
	if(!fg_packet)
		return -2;

	if(SDLNet_UDP_Recv(server_sock, fg_packet) < 1)
		return -1;	// No data received

	p = (char *)fg_packet->data;
	if (!p)
		return -3;	// Null pointer, read failed

	memset(&params, 0, sizeof(effectParams));

	// Divide the buffer into chunks
	read = sscanf(p, "%d|%f|%f|%f|%f|%f|%f|%f|%f|%f|%d|%f", &reconf,
		      &params.pilot[0], &params.pilot[1], &params.pilot[2],
		      &params.stick[0], &params.stick[1], &params.stick[2],
		      &params.trim[0], &params.trim[1], &params.trim[2],
		      &params.shaker_trigger, &params.rumble_period);

	if (read != 12) {
		printf("Error reading generic I/O!\n");
		return -4;
	}
	//printf("%s\n", p);
	// printf("%d\n", reconf);

/*
	printf("%d|%f|%f|%f|%f|%f|%f|%f|%f|%f|%d|%f\n", reconf,
		      params.pilot[0], params.pilot[1], params.pilot[2],
		      params.stick[0], params.stick[1], params.stick[2],
		      params.trim[0], params.trim[1], params.trim[2],
		      params.shaker_trigger, params.rumble_period);
*/

	memcpy(&new_params, &params, sizeof(effectParams));

	if (reconf)
		reconf_request = reconf;

	return 0;
}

void test_effects(void)
{
	unsigned int start;
	unsigned int runtime = 0.0;
	unsigned int dt;
	bool first_start = true;

	for (int i = 0; i < num_devices; i++) {
		printf("\nTesting device number %d, %s.\n", i + 1, devices[i].name);
		printf("HOLD FIRMLY TO YOUR JOYSTICK DURING THE TEST!\n\n");

		if (devices[i].effectId[SPRING] != -1) {
			signed short strength = 0;

			printf("Press [enter] to start spring test (10000 seconds).\n");
			getchar();

			first_start = true;
			start = SDL_GetTicks();
			do {
				runtime = SDL_GetTicks();
				dt = runtime - start;

				if(dt < 3000) strength = 0;
				else if(dt < 6000) strength = 32760;
				else if(dt < 9000) strength = 16000;
				else strength = 32760;

				printf("Coeff: %d\n", strength);

				devices[i].effect[SPRING].condition.left_coeff[0] = strength;
				devices[i].effect[SPRING].condition.right_coeff[0] = strength;
				devices[i].effect[SPRING].condition.left_coeff[1] = strength;
				devices[i].effect[SPRING].condition.right_coeff[1] = strength;
				devices[i].effect[SPRING].condition.left_coeff[2] = strength;
				devices[i].effect[SPRING].condition.right_coeff[2] = strength;

				reload_effect(&devices[i], &devices[i].effect[SPRING], &devices[i].effectId[SPRING], first_start);

				SDL_Delay(200);
				first_start = false;
			} while (runtime < start + 10000);

			if (devices[i].effectId[SPRING] != -1) SDL_HapticStopEffect(devices[i].device, devices[i].effectId[SPRING]);

		} else
			printf("Skipping spring test: Not supported or effect creation was failed\n");


		if (devices[i].supported & SDL_HAPTIC_CONSTANT) {
			printf("Press [enter] to start constant force test (6 seconds).\n");
			getchar();

			first_start = true;
			start = SDL_GetTicks();
			do {
				runtime = SDL_GetTicks();
				dt = runtime - start;

				float x = cos(3.14159 * dt / 3000.0) * 32760.0;
				float y = sin(3.14159 * dt / 3000.0) * 32760.0;
				float z = sin(3.14159 * dt / 3000.0) * 32760.0;

				if (runtime > start + 6000) {
					x = y = z = 0.0;
				}

				if (devices[i].axes > 0 && devices[i].effectId[CONST_X] != -1) {
					devices[i].effect[CONST_X].constant.level = (signed short)clamp(x, -32760.0, 32760.0);
					reload_effect(&devices[i], &devices[i].effect[CONST_X], &devices[i].effectId[CONST_X], first_start);
				}
				if (devices[i].axes > 1 && devices[i].effectId[CONST_Y] != -1) {
					devices[i].effect[CONST_Y].constant.level = (signed short)clamp(y, -32760.0, 32760.0);
					reload_effect(&devices[i], &devices[i].effect[CONST_Y], &devices[i].effectId[CONST_Y], first_start);
				}
				if (devices[i].axes > 2 && devices[i].effectId[CONST_Z] != -1) {
					devices[i].effect[CONST_Z].constant.level = (signed short)clamp(z, -32760.0, 32760.0);
					reload_effect(&devices[i], &devices[i].effect[CONST_Z], &devices[i].effectId[CONST_Z], first_start);
				}
				first_start = false;
				SDL_Delay(100);
			} while (runtime < start + 6500);

			if (devices[i].effectId[CONST_X] != -1) SDL_HapticStopEffect(devices[i].device, devices[i].effectId[CONST_X]);
			if (devices[i].effectId[CONST_Y] != -1) SDL_HapticStopEffect(devices[i].device, devices[i].effectId[CONST_Y]);
			if (devices[i].effectId[CONST_Z] != -1) SDL_HapticStopEffect(devices[i].device, devices[i].effectId[CONST_Z]);

		} else
			printf("Skipping constant force test: Not supported or effect creation was failed\n");

		if ((devices[i].supported & SDL_HAPTIC_SINE) && devices[i].effectId[STICK_SHAKER] != -1) {
			printf("Press [enter] to start rumble test (2 seconds).\n");
			getchar();

			reload_effect(&devices[i], &devices[i].effect[STICK_SHAKER], &devices[i].effectId[STICK_SHAKER], true);

			start = SDL_GetTicks();
			do {
				runtime = SDL_GetTicks();
				SDL_Delay(100);
			} while (runtime < start + 2000);

			if (devices[i].effectId[STICK_SHAKER] != -1) SDL_HapticStopEffect(devices[i].device, devices[i].effectId[STICK_SHAKER]);

		} else
			printf("Skipping rumble test: Not supported or effect creation was failed\n");

		if (devices[i].effectId[GROUND_NOTCH] != -1) {
			printf("Press [enter] to start ground notch test (6 seconds).\n");
			getchar();

			first_start = true;
			start = SDL_GetTicks();
			do {
				runtime = SDL_GetTicks();
				dt = runtime - start;

				devices[i].effect[GROUND_NOTCH].periodic.period = 100 + (6000 - dt) / 15;

				reload_effect(&devices[i], &devices[i].effect[GROUND_NOTCH], &devices[i].effectId[GROUND_NOTCH], first_start);

				SDL_Delay(100);
				first_start = false;
			} while (runtime < start + 6000);

			if (devices[i].effectId[GROUND_NOTCH] != -1) SDL_HapticStopEffect(devices[i].device, devices[i].effectId[GROUND_NOTCH]);

		} else
			printf("Skipping rumble test: Not supported or effect creation was failed\n");

	}

	printf("\nTest done!\n");
}

/**
 * @brief The entry point of this force feedback demo.
 * @param[in] argc Number of arguments.
 * @param[in] argv Array of argc arguments.
 */
int main(int argc, char **argv)
{
	char *name = NULL;
	effectParams *oldParams = NULL;
	unsigned int runtime = 0;
	unsigned int dt = 0;
	bool test_mode = false;
	Uint32 fgfs_address;
	bool start_effects = true;
	bool old_reconf = false;
	time_t disco_timeout = 0;	// Timeout counter for disconnection


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

	printf("fg-haptic version 0.6\n");
	printf("Force feedback support for Flight Gear\n");
	printf("Copyright 2011, 2014, 2020 Lauri Peltonen, released under GPLv2 or later\n\n");

	if (argc > 1) {
		name = argv[1];
		if ((strcmp(name, "--help") == 0) || (strcmp(name, "-h") == 0)) {
			printf("USAGE: %s [optional parameters]\n"
			       "    -h or --help : Show this help\n"
			       "    -t or --test : Test force feedback effects\n\n"
			       "Telnet port for FlightGear is %d and generic\n"
			       "port is %d. See Readme for details.\n", argv[0], DFLTPORT, DFLTPORT + 1);
			return 0;
		}
		if ((strcmp(name, "--test") == 0) || (strcmp(name, "-t") == 0)) {
			printf("Test mode enabled.\n");
			test_mode = true;
		}
	}

	// Initialize SDL haptics
	init_haptic();

	// Create & upload force feedback effects
	create_effects();

	if (test_mode) {
		test_effects();
		abort_execution(0);
	}

	// Initialize network
	if(SDLNet_Init() < 0) {
		printf("Could not initialize network: %s\n", SDLNet_GetError());
		abort_execution(-1);
	}

	socketset = SDLNet_AllocSocketSet(2);
	if (!socketset) {
		printf("Unable to create socket set: %s\n", SDLNet_GetError());
		abort_execution(-1);
	}
	// Wait for a connection from flightgear generic io
	printf("\n\nWaiting for flightgear generic IO at port %d, please run Flight Gear now!\n", DFLTPORT + 1);
	server_sock = fgfs_udp_reader(DFLTPORT + 1);
	if (!server_sock) {
		printf("Failed to create UDP server!\n");
		abort_execution(-1);
	}

	fgfs_address = fgfs_wait_packet(server_sock);
	if(!fgfs_address) {
		printf("Could not determine fgfs address!\n");
		abort_execution(-1);
	}
	printf("Got connection, sending haptic details through telnet\n");
	printf("Host: %d.%d.%d.%d Port: %d\n", fgfs_address & 0xFF, (fgfs_address >> 8) & 0xFF,
		(fgfs_address >> 16) & 0xFF, (fgfs_address >> 24) & 0xFF, DFLTPORT);

	// Connect to flightgear using telnet
	telnet_sock = fgfsconnect(fgfs_address, DFLTPORT);
	if (!telnet_sock) {
		printf("Could not connect to flightgear with telnet!\n");
		abort_execution(-1);
	}
	// Add sockets to a socket set for polling/selecting
	//SDLNet_TCP_AddSocket(socketset, client_sock);
	SDLNet_TCP_AddSocket(socketset, telnet_sock);

	// Switch to data mode
	fgfswrite(telnet_sock, "data");

	// send the devices to flightgear
	send_devices();

	// allocate memory for old param values
	oldParams = (effectParams *) malloc(num_devices * sizeof(effectParams));
	if (!oldParams) {
		printf("Fatal error: Could not allocate memory!\n");
		abort_execution(-1);
	}

	for (int i = 0; i < num_devices; i++)
		memset((void *)&devices[i].params, 0, sizeof(effectParams));

	printf("Running...\n");

	// Main loop

	disco_timeout = time(NULL);
	while (!quit)		// Loop as long as the connection is alive
	{
		dt = runtime;
		runtime = SDL_GetTicks();	// Run time in ms
		dt = runtime - dt;

		// Back up old parameters
		for (int i = 0; i < num_devices; i++) {
			memcpy((void *)&oldParams[i], (void *)&devices[i].params, sizeof(effectParams));
			memset((void *)&devices[i].params, 0, sizeof(effectParams));
		}

		// Read new parameters
		old_reconf = reconf_request;
		if(!read_fg_generic()) {
			// Valid data received, reset timeout
			disco_timeout = time(NULL);
		}

		// If parameters have changed, apply them
		for (int i = 0; i < num_devices; i++) {
			if (!devices[i].device || !devices[i].open)
				continue;	// Break if device is not opened correctly

			// Ground rumble in normal mode
			devices[i].params.rumble_period = new_params.rumble_period;
			if(devices[i].effectId[GROUND_NOTCH] != -1 && devices[i].rumble_mode == MODE_ALTERNATE) {
				if (new_params.rumble_period > 0.00001) {
					devices[i].effect[GROUND_NOTCH].periodic.period = new_params.rumble_period;
					if(oldParams[i].rumble_period < 0.001) {
						// Start the rumble effect (previously was OFF, now ON
						reload_effect(&devices[i], &devices[i].effect[GROUND_NOTCH], &devices[i].effectId[GROUND_NOTCH], true);
					} else {
						// Update period on running effect
						reload_effect(&devices[i], &devices[i].effect[GROUND_NOTCH], &devices[i].effectId[GROUND_NOTCH], false);
					}
				} else if(oldParams[i].rumble_period > 0.001) {
					// Stop rumble effect on liftoff (previously effect was ON, now OFF)
					SDL_HapticStopEffect(devices[i].device, devices[i].effectId[GROUND_NOTCH]);
				}
			}

			// Constant forces (stick forces, pilot G forces
			if ((devices[i].supported & SDL_HAPTIC_CONSTANT)) {
				// Pilot forces
				for(int ax = 0; ax < AXES; ax++) {
					if (devices[i].pilot_axes[ax] >= 0 && devices[i].pilot_axes[ax] < devices[i].axes)
						devices[i].params.force.f[devices[i].pilot_axes[ax]] += new_params.pilot[ax] * devices[i].pilot_gain * devices[i].pilot_invert[ax];
				}

				if (devices[i].stick_mode == MODE_ALTERNATE) {
					// Stick forces with axis mapping
					for(int ax = 0; ax < AXES; ax++) {
						if (devices[i].stick_axes[ax] >= 0 && devices[i].stick_axes[ax] < devices[i].axes)
							devices[i].params.force.f[devices[i].stick_axes[ax]] += new_params.stick[ax] * devices[i].stick_gain * devices[i].stick_invert[ax];
					}
				}

				for(int ax = 0; ax < AXES; ax++)
					devices[i].params.force.f[ax] *= 32760.0;

				// Low pass filter
				float g1 = ((float)dt / (devices[i].lowpass + dt));
				float g2 = (devices[i].lowpass / (devices[i].lowpass + dt));
				for(int ax = 0; ax < AXES; ax++)
					devices[i].params.force.f[ax] = devices[i].params.force.f[ax] * g1 + oldParams[i].force.f[ax] * g2;


				// Add ground rumble in alternate mode
				float rumble = 0.0;
				if (new_params.rumble_period > 0.001 && devices[i].rumble_mode == MODE_NORMAL) {
					// Add the rumble to constant force every now and then
					if ((runtime - devices[i].last_rumble) > new_params.rumble_period) {
						rumble = devices[i].rumble_gain * 32760.0;
						devices[i].last_rumble = runtime;
					}
				}

				// Apply the force
				if (devices[i].effectId[CONST_X] != -1) {
					devices[i].effect[CONST_X].constant.level =
					    (signed short)clamp(devices[i].params.force.d.x, -32760.0, 32760.0);
					reload_effect(&devices[i], &devices[i].effect[CONST_X], &devices[i].effectId[CONST_X], start_effects);
				}
				if (devices[i].effectId[CONST_Y] != -1) {
					devices[i].effect[CONST_Y].constant.level =
					    (signed short)clamp(devices[i].params.force.d.y + rumble, -32760.0, 32760.0);
					reload_effect(&devices[i], &devices[i].effect[CONST_Y], &devices[i].effectId[CONST_Y], start_effects);
				}
				if (devices[i].effectId[CONST_Z] != -1) {
					devices[i].effect[CONST_Z].constant.level =
					    (signed short)clamp(devices[i].params.force.d.z, -32760.0, 32760.0);
					reload_effect(&devices[i], &devices[i].effect[CONST_Z], &devices[i].effectId[CONST_Z], start_effects);
				}
				//printf("dt: %d  X: %.6f  Y: %.6f\n", (unsigned int)dt, devices[i].params.x, devices[i].params.y);
				//printf("dt: %d  X: %d  Y: %d\n", (unsigned int)dt,  devices[i].effect[CONST_X].constant.level, devices[i].effect[CONST_Y].constant.level);
			}

			// Spring effect if stick forces in normal mode
			if (devices[i].effectId[SPRING] != -1 && devices[i].stick_mode == MODE_NORMAL) {
				// Low pass filter coefficients
				float g1 = ((float)dt / (devices[i].lowpass + dt));
				float g2 = (devices[i].lowpass / (devices[i].lowpass + dt));

				// Set center point according to trim
				for(int j=0; j<AXES; j++) {
					int ax = devices[i].stick_axes[j];

					if (ax < 0 || ax >= AXES)
						continue;

					devices[i].effect[SPRING].condition.center[ax] = (signed short)clamp(new_params.trim[j]*32767, -32760, 32760);

					// Calculate the new coefficient
					devices[i].params.coeff[ax] = new_params.stick[j] * devices[i].stick_gain * devices[i].stick_invert[j];

					// Apply low pass filter
					devices[i].params.coeff[ax] = devices[i].params.coeff[ax] * g1 + oldParams[i].coeff[ax] * g2;

					// Note! the axis are inverted when they come from flightgear, so invert here again (???)
					devices[i].effect[SPRING].condition.left_coeff[ax] =
						(signed short)clamp(devices[i].params.coeff[ax] * 32767, -32760, 32760);
					devices[i].effect[SPRING].condition.right_coeff[ax] =
						(signed short)clamp(devices[i].params.coeff[ax] * 32767, -32760, 32760);
				}

				reload_effect(&devices[i], &devices[i].effect[SPRING], &devices[i].effectId[SPRING], start_effects);
				//printf("dt: %d  X: %d  Y: %d\n", (unsigned int)dt,  devices[i].effect[SPRING].condition.left_coeff[0], devices[i].effect[SPRING].condition.left_coeff[1]);
			}


			// Stick shaker trigger
			if (devices[i].effectId[STICK_SHAKER] != -1) {
				if (new_params.shaker_trigger && !oldParams[i].shaker_trigger) {
					reload_effect(&devices[i], &devices[i].effect[STICK_SHAKER], &devices[i].effectId[STICK_SHAKER], true);
				} else if (!new_params.shaker_trigger && oldParams[i].shaker_trigger) {
					SDL_HapticStopEffect(devices[i].device, devices[i].effectId[STICK_SHAKER]);
				}
				devices[i].params.shaker_trigger = new_params.shaker_trigger;
			}

			// Launch friction
			if (devices[i].effectId[FRICTION] != -1 && start_effects) {
				reload_effect(&devices[i], &devices[i].effect[FRICTION], &devices[i].effectId[FRICTION], true);
			}

			// Launch damping
			if (devices[i].effectId[DAMPING] != -1 && start_effects) {
				reload_effect(&devices[i], &devices[i].effect[DAMPING], &devices[i].effectId[DAMPING], true);
			}

			start_effects = false;
		}

		// Reconfigure the system if reconfiguration was requested
		//printf("%d %d\n", reconf_request, old_reconf);
		if (reconf_request && !old_reconf) {
			// Stop all effects from playing when reconfiguring
			for(int i = 0; i < num_devices; i++)
				SDL_HapticStopAll(devices[i].device);

			if(reconf_request > 1)
				send_devices();

			reconf_request = 0;
			read_devices();
			create_effects();
			start_effects = true;

			disco_timeout = time(NULL);	// Reset disconnection timeout
		}

		if((time(NULL) - disco_timeout) > DISCONNECT_TIMEOUT) {
			// No data for a while, assume fgfs is closed and wait for new connection
			printf("FlightGear disconnected! Waiting for reconnection...\n");
			fgfsclose(telnet_sock);

			while(1) {
				fgfs_address = fgfs_wait_packet(server_sock);
				if(fgfs_address) break;
			}

			printf("Got connection, sending haptic details through telnet\n");
			printf("Host: %d.%d.%d.%d Port: %d\n", fgfs_address & 0xFF, (fgfs_address >> 8) & 0xFF,
					(fgfs_address >> 16) & 0xFF, (fgfs_address >> 24) & 0xFF, DFLTPORT);

			// Connect to flightgear using telnet
			telnet_sock = fgfsconnect(fgfs_address, DFLTPORT);
			if (!telnet_sock) {
				printf("Could not connect to flightgear with telnet!\n");
				abort_execution(-1);
			}
			// Switch to data mode
			fgfswrite(telnet_sock, "data");

			// send the devices to flightgear
			send_devices();

			printf("Reconnected!\n");
			disco_timeout = time(NULL);
		}

		SDL_Delay(10);
	}

	// Close flightgear telnet connection
	fgfswrite(telnet_sock, "quit");
	fgfsclose(telnet_sock);

	// Close generic connection
	//fgfsclose(client_sock);
	//fgfsclose(server_sock);
	SDLNet_UDP_Close(server_sock);

	if(socketset) {
		SDLNet_FreeSocketSet(socketset);
		socketset = NULL;
	}

	if (oldParams)
		free(oldParams);
	oldParams = NULL;

	// Close haptic devices
	for (int i = 0; i < num_devices; i++)
		if ( /*devices[i].open && */ devices[i].device)
			SDL_HapticClose(devices[i].device);

	if (devices)
		free(devices);
	devices = NULL;

	SDL_Quit();

	return 0;
}

/*
 * Cleans up a bit.
 */
void abort_execution(int signal)
{
	printf("\nAborting program execution.\n");

	// Close flightgear telnet connection
	fgfswrite(telnet_sock, "quit");
	fgfsclose(telnet_sock);

	if(!fg_packet)
		SDLNet_FreePacket(fg_packet);


	// Adn generic
	//fgfsclose(client_sock);
	//fgfsclose(server_sock);
	SDLNet_UDP_Close(server_sock);

	if(socketset) {
		SDLNet_FreeSocketSet(socketset);
		socketset = NULL;
	}

	// Close haptic devices
	for (int i = 0; i < num_devices; i++)
		if (devices[i].open && devices[i].device)
			SDL_HapticClose(devices[i].device);

	if (devices)
		free(devices);
	devices = NULL;

	SDLNet_Quit();

	SDL_Quit();

	exit(1);
}

/*
 * Displays information about the haptic device.
 */
void HapticPrintSupported(SDL_Haptic * haptic)
{
	unsigned int supported;

	supported = SDL_HapticQuery(haptic);
	printf("   Device has %d axis\n", SDL_HapticNumAxes(haptic));
	printf("   Supported effects [%d effects, %d playing]:\n", SDL_HapticNumEffects(haptic), SDL_HapticNumEffectsPlaying(haptic));
	if (supported & SDL_HAPTIC_CONSTANT)
		printf("      constant\n");
	if (supported & SDL_HAPTIC_SINE)
		printf("      sine\n");
/*    if (supported & SDL_HAPTIC_SQUARE)
        printf("      square\n");*/
	if (supported & SDL_HAPTIC_TRIANGLE)
		printf("      triangle\n");
	if (supported & SDL_HAPTIC_SAWTOOTHUP)
		printf("      sawtoothup\n");
	if (supported & SDL_HAPTIC_SAWTOOTHDOWN)
		printf("      sawtoothdown\n");
	if (supported & SDL_HAPTIC_RAMP)
		printf("      ramp\n");
	if (supported & SDL_HAPTIC_FRICTION)
		printf("      friction\n");
	if (supported & SDL_HAPTIC_SPRING)
		printf("      spring\n");
	if (supported & SDL_HAPTIC_DAMPER)
		printf("      damper\n");
	if (supported & SDL_HAPTIC_INERTIA)
		printf("      intertia\n");
	if (supported & SDL_HAPTIC_CUSTOM)
		printf("      custom\n");
	printf("   Supported capabilities:\n");
	if (supported & SDL_HAPTIC_GAIN)
		printf("      gain\n");
	if (supported & SDL_HAPTIC_AUTOCENTER)
		printf("      autocenter\n");
	if (supported & SDL_HAPTIC_STATUS)
		printf("      status\n");
}

int fgfswrite(TCPsocket sock, char *msg, ...)
{
	va_list va;
	ssize_t len;
	char buf[MAXMSG];

	if (!sock)
		return 0;

	va_start(va, msg);
	vsnprintf(buf, MAXMSG - 2, msg, va);
	va_end(va);
	//printf("SEND: \t<%s>\n", buf);
	strcat(buf, "\r\n");

	len = SDLNet_TCP_Send(sock, buf, strlen(buf));
	if (len < 0) {
		printf("Error in fgfswrite: %s\n", SDLNet_GetError());
		exit(EXIT_FAILURE);
	}
	return len;
}

const char *fgfsread(TCPsocket sock, int timeout)
{
	static char buf[MAXMSG];
	char *p = &buf[0];
	size_t len = 0;
	int ready = 0;
	time_t start;

	memset(buf, 0, MAXMSG);

	start = time(NULL);
	do {
		ready = SDLNet_CheckSockets(socketset, timeout * 1000);
		if (!ready) {
			//printf("Timeout!\n");
			return NULL;
		}
		SDL_Delay(5);

		// TODO: Loop until socket is ready or timeout?
		if (SDLNet_SocketReady(sock)) {
			ready = 1;
		}
	} while (!ready && ((time(NULL) - start) < timeout));

	if (!ready) {
		//printf("Error in fgfsread: Socket was not ready (timeout)!\n");
		return NULL;
	}

	ready = 0;
	do {
		if (SDLNet_TCP_Recv(sock, p, 1) <= 0) {
			// printf("Error in fgfsread: Recv returned zero!\n");
			printf("Telnet connection closed. Quit!\n");
			quit = true;
			return NULL;
		}
		len++;
		if (len == MAXMSG - 1) {
			printf("Warning in fgfsread: Buffer size exceeded!\n");
			ready = 1;
		} else if (*p == '\n') {
			ready = 1;
		}
		p++;
	} while (!ready);

	for (p = &buf[len - 1]; p >= buf; p--)
		if (*p != '\r' && *p != '\n')
			break;
	*++p = '\0';

	// if(strlen(buf)) printf("RECV: %s\n", buf);

	return strlen(buf) ? buf : NULL;
}

void fgfsflush(TCPsocket sock)
{
	const char *p;
	while ((p = fgfsread(sock, 0)) != NULL) {
		//printf("IGNORE: \t<%s>\n", p);
	}
}

TCPsocket fgfsconnect(const Uint32 host, const Uint16 port)
{
	IPaddress serv_addr, cli_addr;
	TCPsocket _sock, _clientsock;
	time_t start;

	if (host)		// Act as a client -> connect to address
	{
		char hostname[32] = {0};
		snprintf(hostname, 31, "%d.%d.%d.%d", host & 0xFF, (host >> 8) & 0xFF, (host >> 16) & 0xFF, (host >> 24) & 0xFF);
		if (SDLNet_ResolveHost(&cli_addr, hostname, port) == -1) {
			printf("Error in fgfsconnect, resolve host: %s\n", SDLNet_GetError());
			return NULL;
		}

		_sock = SDLNet_TCP_Open(&cli_addr);
		if (!_sock) {
			printf("Error in fgfsconnect, connect: %s\n", SDLNet_GetError());
			return NULL;
		}

		return _sock;

	} else {		// Act as a server, wait for connections
		if (SDLNet_ResolveHost(&serv_addr, NULL, port) == -1) {
			printf("Error in fgfsconnect, server resolve host: %s\n", SDLNet_GetError());
			return NULL;
		}

		_sock = SDLNet_TCP_Open(&serv_addr);
		if (!_sock) {
			printf("Error in fgfsconnect, server connect: %s\n", SDLNet_GetError());
			return NULL;
		}
		// Wait for connection until timeout
		start = time(NULL);
		do {
			SDL_Delay(50);
			_clientsock = SDLNet_TCP_Accept(_sock);
		} while (!_clientsock && ((time(NULL) - start) < CONN_TIMEOUT));

		if (!_clientsock) {
			printf("Error in fgfsconnect: Connection timeout\n");
			return NULL;
		}

		//client_sock = _clientsock;

		return _sock;
	}
}

UDPsocket fgfs_udp_reader(const Uint16 port)
{
	UDPsocket _sock;

	_sock = SDLNet_UDP_Open(port);
	if(!_sock) {
		printf("Error in fgfs_udp_reader, open: %s\n", SDLNet_GetError());
		return NULL;
	}

	return _sock;
}

Uint32 fgfs_wait_packet(UDPsocket socket)
{
	time_t start;
	UDPpacket *p;
	int ret;
	Uint32 host;

	p = SDLNet_AllocPacket(512);
	if(!p) {
		printf("Memory allocation error in fgfs_wait_packet: %s\n", SDLNet_GetError());
		return 0;
	}

	start = time(NULL);
	do {
		SDL_Delay(50);
		ret = SDLNet_UDP_Recv(socket, p);
	} while (!ret && ((time(NULL) - start) < CONN_TIMEOUT));

	if(!ret) {
		printf("Timeout in fgfs_wait_packet\n");
		SDLNet_FreePacket(p);
		return 0;
	}

	host = p->address.host;

	SDLNet_FreePacket(p);

	return host;
}
