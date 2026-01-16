//////////////////////////////////////////////////////////////////////////////////////////////////////
//  File to convert log.csv file from avionics to:                                                  //
//  LogAvgData.csv (Time, Temp(SHT4x and BMP280), Average Temp, Pressure and Altitude)              //
//  LogMotionData.CSV (Time, Accelerations and Gyro Accelerations (Corrected to global Direction))  //
//  ekf.csv (Roll, Pitch, Yaw, Velocites and Positions in global frame)                             //
//      -> Used to plot 3D Graph via .m File                                                        //
//////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

//Max data lines from the log
#define maxData 100000

typedef struct { //struct to store sensor data
  uint32_t time; //TIme of log in ms
  float humidity; //Humidity in %
  float temp_sens; //Temp in C from SHT4x
  float temp_bmp; //Temp in C from BMP180
  float pressure; //Pressure in kPa from BMP180
  float altitude; //Altitude in M from BMP180
  float ax; //Acceleration X in m/s^2
  float ay; //Acceleration Y in m/s^2
  float az; //Acceleration Z in m/s^2
  float gx; //Gyro X in deg/s
  float gy; //Gyro Y in deg/s
  float gz; //Gyro Z in deg/s
} Raw_data_struct;

typedef struct { //struct to store average data
  uint32_t time; //TIme of log in ms
  float humidity; //Humidity in %
  float temp_sens; //Temp in C from SHT4x
  float temp_bmp; //Temp in C from BMP180
  float avg_temp; //Average temp in c
  float pressure; //Pressure in kPa from BMP180
  float altitude; //Altitude in M from BMP180
} avgdata;

typedef struct { //struct to store motion data
  uint32_t time; //TIme of log in ms
  float altitude; //Altitude in M from BMP180
  float ax; //Acceleration X in m/s^2
  float ay; //Acceleration Y in m/s^2
  float az; //Acceleration Z in m/s^2
  float gx; //Gyro X in deg/s
  float gy; //Gyro Y in deg/s
  float gz; //Gyro Z in deg/s
} motiondata;

typedef struct { //Struct to store EKF values
    float X[9];      // [roll, pitch, yaw, vx, vy, vz, x, y, z]
    float P[9][9];   // state covariance
    float Q[9][9];   // process noise covariance
    float R;         // measurement noise for altitude
} ekf_state;

uint8_t fileFind(char* fileName, uint8_t *index); //Find file name
uint8_t importData(Raw_data_struct *rawData, char *fileName, uint32_t *count); //Import data
uint8_t fillAvgData(uint32_t dataPoints, Raw_data_struct *rawData, avgdata *avgData); //Fill average data struct
uint8_t writeAvgData(avgdata *avgData, uint8_t index, uint32_t dataPoints); //Write average data file
uint8_t fillMotionData(uint32_t dataPoints, Raw_data_struct *rawData, motiondata *motionData); //Fill motion data file
uint8_t writeMotionData(motiondata *motionData, uint8_t index, uint32_t dataPoints); //Write motion data file
void ekf_init(ekf_state *state, const motiondata *m); //Initialise Extended Kalman Filter
void body_to_world(float roll, float pitch, float yaw, //Convert relative to world position
                   float ax, float ay, float az,
                   float *wx, float *wy, float *wz);
void ekf_predict(ekf_state *state, const motiondata *m, float dt); //EKF prediction
void ekf_update_altitude(ekf_state *ekf, float alt_measure); //EKF update wioth barometer altiude

int main() { //Main function
    char fileName[20]; //Filename store for log.csv
    uint8_t index = 1; //Index for log iteration
    if (fileFind(fileName,&index) == 0) {return 0;} //Call function to find and return file name
    uint32_t dataPoints=0; //Number of data points
    Raw_data_struct rawData[maxData];  //Create raw data struct
    if (importData(rawData,fileName,&dataPoints) == 0) {return 0;} //Import data

    avgdata *avgData = malloc(dataPoints * sizeof(*avgData)); //Allocate memory for avgData and motionData
    motiondata *motionData = malloc(dataPoints * sizeof(*motionData));
    if (!avgData || !motionData) { //Error if allocation fail
        printf("ERROR: Memory allocation failed\n");
        return 0;
    }

    if (fillAvgData(dataPoints,rawData,avgData) == 0) {return 0;} //Fill average data struct from raw with calculations
    if (writeAvgData(avgData,index,dataPoints) == 0) {return 0;} //Writes the average data file
    free(avgData); //Unallocates memory

    if (fillMotionData(dataPoints,rawData,motionData) == 0) {return 0;} //Same as above but for motion data
    if (writeMotionData(motionData,index,dataPoints) == 0) {return 0;}

    ekf_state ekf; //Create EKF struct
    ekf_init(&ekf, &motionData[0]); //Initialise EKF

    char ekfFileName[50]; //Get ekf File name
    sprintf(ekfFileName,"ekf%d.csv",index);
    FILE *fekf = fopen(ekfFileName,"w"); //Open file for writing
    if (!fekf) { //If it fails to open, stop
        printf("ERROR: Failed to open ekf.csv\n");
        return 0;
    }
    fprintf(fekf,"Roll_R,Pitch_R,Yaw_R,Vx,Vy,Vz,X,Y,Z\n"); //print headers

    for (uint32_t i = 1; i < dataPoints; i++) { //Repeat for all data points
        float dt = (motionData[i].time - motionData[i-1].time) / 1000.0f; //Get change in time between points

        // Predict step
        ekf_predict(&ekf, &motionData[i], dt);

        // Update step: fuse barometer altitude
        ekf_update_altitude(&ekf, motionData[i].altitude);


        // Write EKF full state to CSV
        fprintf(fekf,
            "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            ekf.X[0], // roll
            ekf.X[1], // pitch
            ekf.X[2], // yaw
            ekf.X[3], // vx
            ekf.X[4], // vy
            ekf.X[5], // vz
            ekf.X[6], // x
            ekf.X[7], // y
            ekf.X[8]  // z / altitude
        );

    }

    fclose(fekf); //Close File
    free(motionData); //Clear alloacted memory
    printf("Finished\n"); //Print finished statement
}


uint8_t fileFind(char* fileName, uint8_t *index){ //Find file function
    if (fileName == NULL || index ==NULL) { //If invalid inputs, return error
        printf("ERROR: Invalid fileFind inputs\n");
        return 0;
    }
    char temp_name[20]; //Temp variable storage
    while(1){ //Repeat till error or found
        if (*index == 255) { //If index exceed 255, return error
            printf("ERROR: File not found!\n");
            return 0;
        }
        sprintf(temp_name,"log%d.csv",*index); //Get filename string
        FILE *f = fopen(temp_name,"r"); //Attempt open file
        if (f){ //If opens
            strcpy(fileName,temp_name); //Copy string
            fclose(f); //Close file
            printf("File found\n");
            return 1;
        }
        (*index)++; //Increase index if not opened
    }
}

uint8_t importData(Raw_data_struct *rawData, char *fileName, uint32_t *count){ //Import data function
    if (rawData == NULL || fileName == NULL) { //If inputs are invalid, stop code
        printf("ERROR: Invalid importData inputs\n");
        return 0;
    }
    char line[1024]; //Temp variable to store line of csv
    FILE *fptr = fopen(fileName,"r"); //Open file for reading
    if (!fptr) { //If not opened return error
        printf("ERROR: Failed to open file\n");
        return 0;
    }
    fgets(line,sizeof(line),fptr); //Get header line from csv
    while(fgets(line,sizeof(line),fptr)) { //Repeat for lines with numbers
        if (*count>=maxData){ //If the size of file is too large stop inputting data
            printf("ERROR: Too much Data");
            return 0;
        }
        uint8_t n = sscanf( //Move data from temp line to corresponding struct location

            line,
            "%u,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
            &rawData[*count].time,
            &rawData[*count].humidity,
            &rawData[*count].temp_sens,
            &rawData[*count].temp_bmp,
            &rawData[*count].pressure,
            &rawData[*count].altitude,
            &rawData[*count].ax,
            &rawData[*count].ay,
            &rawData[*count].az,
            &rawData[*count].gx,
            &rawData[*count].gy,
            &rawData[*count].gz
        );
        if (n != 12) { //If unsucessful read
            printf("ERROR: Failed to read file\n");
            return 0;
        }
        (*count)++;  //Increase count
    }
    printf("Finished Reading File\n");
    return 1;
}

uint8_t fillAvgData(uint32_t dataPoints, Raw_data_struct *rawData, avgdata *avgData){ //Fill avg data struct
    if (dataPoints == 0 || rawData == NULL || avgData  == NULL) { //If inputs are invalid, stop code
        printf("ERROR: Invalid fillAvgData inputs\n");
        return 0;
    }
    for (uint32_t i=0; i<dataPoints; i++) { //Get corresponding data from raw to average
        avgData[i].time = rawData[i].time;
        avgData[i].humidity = rawData[i].humidity;
        avgData[i].temp_sens = rawData[i].temp_sens;
        avgData[i].temp_bmp = rawData[i].temp_bmp;
        avgData[i].avg_temp = (rawData[i].temp_bmp + rawData[i].temp_sens)/2; //Calculate average data
        avgData[i].pressure = rawData[i].pressure;
        avgData[i].altitude = rawData[i].altitude;
    }
    printf("Finished filling avgData\n");
    return 1;
}

uint8_t writeAvgData(avgdata *avgData, uint8_t index, uint32_t dataPoints){ //Write average data to file
    if (avgData == NULL) { //If data struct is invalif, stop code
        printf("ERROR: Invalid writeAvgData inputs\n");
        return 0;
    }
    char avgDataFileName[50]; //File name temp
    sprintf(avgDataFileName,"LogAvgData%d.csv",index); //Create filename from previous index, so number aligns
    FILE *favg = fopen(avgDataFileName,"w"); //Open file
    if (!favg) { //If not opened, stop code
        printf("ERROR: Failed to open LogAvgData.csv\n");
        return 0;
    }
    fprintf(favg,"Time_ms,Humidity_%%,Temp_C,Temp_BMP_C,Avg_Temp_C,Pressure_kPa,Altitude_m\n"); //Print header
    for (uint32_t i=0; i<dataPoints; i++) { //Print line for each datapoint
        fprintf(favg,
            "%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            avgData[i].time,
            avgData[i].humidity,
            avgData[i].temp_sens,
            avgData[i].temp_bmp,
            avgData[i].avg_temp,
            avgData[i].pressure,
            avgData[i].altitude
        );
    }
    fclose(favg); //Close file
    printf("Finished Writing LogAvgData.csv\n");
    return 1;
}

uint8_t fillMotionData(uint32_t dataPoints, Raw_data_struct *rawData, motiondata *motionData){ //Same as avgDataFill
    if (dataPoints <= 0 || rawData == NULL || motionData  == NULL) {
        printf("ERROR: Invalid fillMotionData inputs\n");
        return 0;
    }
    for (uint32_t i=0; i<dataPoints; i++) {
        motionData[i].time = rawData[i].time;
        motionData[i].altitude = rawData[i].altitude;
        motionData[i].ax = -rawData[i].az; //Converts from chip realtive orientation to world orientation
        motionData[i].ay = -rawData[i].ax;
        motionData[i].az = -rawData[i].ay;
        motionData[i].gx = -rawData[i].gz;
        motionData[i].gy = -rawData[i].gx;
        motionData[i].gz = -rawData[i].gy;
    }
    printf("Finished filling motionData\n");
    return 1;
}

uint8_t writeMotionData(motiondata *motionData, uint8_t index, uint32_t dataPoints){ //Same as write avgDataFile
    if (motionData == NULL) {
        printf("ERROR: Invalid writeMotionData inputs\n");
        return 0;
    }
    char motionDataFileName[50];
    sprintf(motionDataFileName,"LogMotionData%d.csv",index);
    FILE *f = fopen(motionDataFileName,"w");
    if (!f) {
        printf("ERROR: Failed to open LogMotionData.csv\n");
        return 0;
    }
    fprintf(f,"Time_ms,Altitude_M,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z\n");
    for (uint32_t i=0; i<dataPoints; i++) {
        fprintf(f,
            "%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            motionData[i].time,
            motionData[i].altitude,
            motionData[i].ax,
            motionData[i].ay,
            motionData[i].az,
            motionData[i].gx,
            motionData[i].gy,
            motionData[i].gz
        );
    }
    fclose(f);
    printf("Finished Writing LogMotionData.csv\n");
    return 1;
}

void ekf_init(ekf_state *ekf, const motiondata *m) { //Initialtes EKF
    // Initialize state
    for (int i = 0; i < 9; i++) {
        ekf->X[i] = 0.0f;
        for (int j = 0; j < 9; j++)
            ekf->P[i][j] = (i == j) ? 0.01f : 0.0f; // initial covariance
    }
    ekf->X[8] = m->altitude; // initialize z

    // Clear Q
    for (int i = 0; i < 9; i++){
        for (int j = 0; j < 9; j++){
            ekf->Q[i][j] = 0.0f;
        }
    }   
    // Orientation
    ekf->Q[0][0] = 0.0005f;  // roll
    ekf->Q[1][1] = 0.0005f;  // pitch
    ekf->Q[2][2] = 0.0008f;  // yaw (gyro bias drift slightly higher)

    // Velocities
    ekf->Q[3][3] = 0.02f;    // vx
    ekf->Q[4][4] = 0.02f;    // vy
    ekf->Q[5][5] = 0.015f;   // vz

    // Positions
    ekf->Q[6][6] = 0.0002f;  // x
    ekf->Q[7][7] = 0.0002f;  // y
    ekf->Q[8][8] = 0.0002f;  // z / altitude



    // Measurement noise variance for altitude
    ekf->R = 0.2f;
}


void body_to_world(float roll, float pitch, float yaw,
                   float ax, float ay, float az,
                   float *wx, float *wy, float *wz){ //Converts from realtive velocity to world frame
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    *wx = cp*cy*ax + (sr*sp*cy - cr*sy)*ay + (cr*sp*cy + sr*sy)*az;
    *wy = cp*sy*ax + (sr*sp*sy + cr*cy)*ay + (cr*sp*sy - sr*cy)*az;
    *wz = -sp*ax   + sr*cp*ay              + cr*cp*az;
}

void ekf_predict(ekf_state *ekf, const motiondata *m, float dt) { //EKF prediction step
    const float DEG2RAD = 0.0174532925f;

    // 1. Integrate gyro → orientation (roll, pitch, yaw)
    ekf->X[0] += m->gx * DEG2RAD * dt;  // roll
    ekf->X[1] += m->gy * DEG2RAD * dt;  // pitch
    ekf->X[2] += m->gz * DEG2RAD * dt;  // yaw

    // 2. Convert body acceleration to world frame
    float ax_w, ay_w, az_w;
    body_to_world(ekf->X[0], ekf->X[1], ekf->X[2],
                  m->ax, m->ay, m->az,
                  &ax_w, &ay_w, &az_w);

    // 3. Remove gravity
    az_w -= 9.81f;

    // 4. Integrate velocity
    ekf->X[3] += ax_w * dt;  // vx
    ekf->X[4] += ay_w * dt;  // vy
    ekf->X[5] += az_w * dt;  // vz

    // 5. Integrate position
    ekf->X[6] += ekf->X[3] * dt; // x
    ekf->X[7] += ekf->X[4] * dt; // y
    ekf->X[8] += ekf->X[5] * dt; // z / altitude

    // 6. Update covariance: P = P + Q*dt
    for (int i = 0; i < 9; i++)
        for (int j = 0; j < 9; j++)
            ekf->P[i][j] += ekf->Q[i][j] * dt;

    // 7. Optional: small cross-axis damping for stability
    ekf->P[3][4] *= 0.99f;  // vx-vy correlation damping
    ekf->P[3][5] *= 0.99f;  // vx-vz correlation damping
    ekf->P[4][5] *= 0.99f;  // vy-vz correlation damping
    ekf->P[4][3] = ekf->P[3][4];  // symmetry
    ekf->P[5][3] = ekf->P[3][5];
    ekf->P[5][4] = ekf->P[4][5];
    ekf->P[3][3] *= 0.999f;
    ekf->P[4][4] *= 0.999f;
    ekf->P[5][5] *= 0.999f;

}

void ekf_update_altitude(ekf_state *ekf, float alt_measure) { //Update EKF with barometer reading
    // Measurement residual: z_meas - z_pred
    float y = alt_measure - ekf->X[8];

    // Kalman gain: K = P[:,8] / (P[8][8] + R)
    float K[9];
    for (int i = 0; i < 9; i++)
        K[i] = ekf->P[i][8] / (ekf->P[8][8] + ekf->R);

    // Update state with altitude measurement
    for (int i = 0; i < 9; i++)
        ekf->X[i] += K[i] * y;

    // Update covariance: P = P - K * P[8,:]
    for (int i = 0; i < 9; i++)
        for (int j = 0; j < 9; j++)
            ekf->P[i][j] -= K[i] * ekf->P[8][j];
}
