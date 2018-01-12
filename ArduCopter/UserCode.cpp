#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>

#include "Copter.h"
#include "pid.h"

int x_error;
int y_error;
float distance_x;
float distance_x_2;
float distance_y;
float distance_y_2;

int server_socket;
int client_socket;
char client_mess[256];
char client_mess_x[5];
char client_mess_y[5];

PID pid_roll;
PID pid_pitch;

void server_init(void);
bool decode (char* msg, int & x_error,int & y_error);


#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    pid_roll.pid_set_k_params(0.1, 0, 0.125, 0.01, 1000);
    pid_pitch.pid_set_k_params(0.1, 0, 0.125, 0.01, 1000);
    server_init();  
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    int X_err_in_pixel = 0, Y_err_in_pixel = 0;
    client_socket = accept(server_socket, NULL, NULL);
    recv(client_socket, &client_mess, sizeof(client_mess), 0);
    bool isThereaAnyObject = decode(client_mess, X_err_in_pixel, Y_err_in_pixel);

    decode(client_mess, X_err_in_pixel, Y_err_in_pixel);

    // cliSerial->printf("%f %f %f\n", curr_roll, curr_pitch, curr_height);
    // cliSerial->printf("client mess: %s \n",client_mess);
    // cliSerial->printf("err in pixel %d %d \n", X_err_in_pixel, Y_err_in_pixel);

    // Process information
    if (isThereaAnyObject && curr_roll < MAX_ANGEL && curr_roll >- MAX_ANGEL && curr_pitch < MAX_ANGEL && curr_pitch > -MAX_ANGEL ){
        pixel_per_cm = curr_height * 0.8871428438 * 2 / 800;
        X_err_in_cm = X_err_in_pixel * pixel_per_cm;
        Y_err_in_cm = Y_err_in_pixel * pixel_per_cm;

        roll_error_x = pid_roll.pid_process(distance_x_2, millis());
        pitch_error_y = pid_pitch.pid_process(distance_y_2, millis());
    }
    else{
        roll_error_x = 0;
        pitch_error_y = 0;
    }





        // cliSerial->printf("errX: %d errY: %d \r\n", x_error, y_error);
    
        // calculate the real distance, x:roll, y: pitch
        // double t_x = fabs(tan(beta_x - ahrs.roll) - tan(ahrs.roll + beta_x));
        // double t_y = fabs(tan(beta_y - ahrs.roll) - tan(ahrs.roll + beta_y));
        // int height = rangefinder.distance_cm_orient(ROTATION_PITCH_270);

        // // random cases
        // double cm_per_pixel_x = height*tan(beta_x - ahrs.roll)/ ((640*tan(beta_x - ahrs.roll))/ (tan(beta_x + ahrs.roll) + tan(beta_x - ahrs.roll)));
        // double cm_per_pixel_y = height*tan(beta_y - ahrs.pitch)/((480*tan(beta_y - ahrs.pitch))/(tan(beta_y + ahrs.pitch) + tan(beta_y - ahrs.pitch)));
        
        // calculate the distance in cm to x axis
        // if(x_error > 0)
        //     distance_x = x_error*cm_per_pixel_x+ height*t_x/2;
        // else if(x_error < -height*t_x/2)
        //     distance_x = x_error*cm_per_pixel_x- height*t_x/2;
        // else distance_x = -x_error*cm_per_pixel_x+ height*t_x/2;

        // // calculate the distance in cm to y axis
        // if(y_error > 0)
        //     distance_y = y_error*cm_per_pixel_y + height*t_y/2;
        // else if(y_error < -height*t_y/2)
        //     distance_y = y_error*cm_per_pixel_y - height*t_y/2;
        // else distance_y = -y_error*cm_per_pixel_y + height*t_y/2; 

        //case camera is perpendicular to the ground
        // int height = rangefinder.distance_cm_orient(ROTATION_PITCH_270);
        // float cm_per_pixel_x_2 = height*tan_beta_x*2/640;
        // float cm_per_pixel_y_2 = height*tan_beta_y*2/480;
        // distance_x_2 = x_error*cm_per_pixel_x_2;
        // distance_y_2 = y_error*cm_per_pixel_y_2;

        // cliSerial->printf("cm_per_pixel_x: %f  cm_per_pixel_y: %f  \r\n", cm_per_pixel_x_2,cm_per_pixel_y_2);
        // cliSerial->printf("errX: %d errY: %d  dis_x: %3.5f dis_x_2: %3.5f \r\n", x_error, y_error, distance_x_2,distance_y_2);

        // roll_error = pid_roll.pid_process(distance_x_2, millis());
        // pitch_error = pid_pitch.pid_process(distance_y_2, millis());
        // cliSerial->printf("roll_error: %d  pitch_error: %d  \r\n", roll_error,pitch_error);
    }

}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 20Hz code here
//==============================TEMPERATURE======================================//
    air_temperature = barometer.get_temperature();
    // hal.uartF->printf("temp:%f",air_temperature);

//==============================IPS_TRANSMIT======================================//
    hal.uartE->printf("{PARAM,TRIGGER_US}\n");
    ips_delay_ms = AP_HAL::millis();    // trigger IPS_transmission on Tiva C
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif


bool server_init(void){
    if(server_socket == 0){
        server_socket = socket(AF_INET, SOCK_STREAM,0);

        //define the server address
        struct sockaddr_in server_address;
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(9000);
        // server_address.sin_addr.s_addr = inet_addr("127.0.0.1");
        server_address.sin_addr.s_addr = INADDR_ANY;

        //bind the socket to our specified IP and port
        bind(server_socket, (struct sockaddr*) &server_address, sizeof(server_address));

        listen(server_socket, 5);

        fcntl(server_socket, F_SETFL, fcntl(server_socket, F_GETFL, 0) | O_NONBLOCK);
    }
    return true;
}

bool decode(char* msg, int& errX, int& errY){
    if (msg[0] == '0')
    {
        errX = 0;
        errY = 0;
        return false;
    }
    else{
        errX = 0;
        errX += (int)(msg[2] - 48) * 100;
        errX += (int)(msg[3] - 48) * 10;
        errX += (int)(msg[4] - 48);
        if (msg[1] == '1') errX = - errX;

        errY = 0;
        errY += (int)(msg[6] - 48) * 100;
        errY += (int)(msg[7] - 48) * 10;
        errY += (int)(msg[8] - 48);
        if (msg[5] == '1')errY = - errY;
        
        return true;
    }
}