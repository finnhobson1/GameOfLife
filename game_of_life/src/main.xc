// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <timer.h>
#include <stdio.h>
#include <limits.h>
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT 16                //image height
#define  IMWD 16                //image width

typedef unsigned char uchar;      //using uchar as shorthand

on tile[0] : in port buttons = XS1_PORT_4E;     //button input port
on tile[0] : out port leds = XS1_PORT_4F;       //LED output port
on tile[0] : port p_scl = XS1_PORT_1E;          //interface ports to orientation
on tile[0] : port p_sda = XS1_PORT_1F;

//register addresses for orientation
#define FXOS8700EQ_I2C_ADDR 0x1E
#define FXOS8700EQ_XYZ_DATA_CFG_REG 0x0E
#define FXOS8700EQ_CTRL_REG_1 0x2A
#define FXOS8700EQ_DR_STATUS 0x0
#define FXOS8700EQ_OUT_X_MSB 0x1
#define FXOS8700EQ_OUT_X_LSB 0x2
#define FXOS8700EQ_OUT_Y_MSB 0x3
#define FXOS8700EQ_OUT_Y_LSB 0x4
#define FXOS8700EQ_OUT_Z_MSB 0x5
#define FXOS8700EQ_OUT_Z_LSB 0x6

char infname[] = "test.pgm";     //input image path
char outfname[] = "testout.pgm"; //output image path

/////////////////////////////////////////////////////////////
//
//Receives pattern from Distributor and displays LED pattern.
//
/////////////////////////////////////////////////////////////
int showLEDs(out port p, chanend fromDist) {
  unsigned int pattern; //1st bit...separate green LED
               //2nd bit...blue LED
               //3rd bit...green LED
               //4th bit...red LED
  while (1) {
    fromDist :> pattern;   //receive new pattern from visualiser
    p <: pattern;                //send pattern to LED port
  }
  return 0;
}

///////////////////////////////////////////////////////////////////
//
//Receives input from buttons port and sends signal to Distributor.
//
///////////////////////////////////////////////////////////////////
void buttonListener(in port b, chanend toDist) {
  int r;
  b when pinseq(15)  :> r;    // check that no button is pressed
  b when pinsneq(15) :> r;    // check if some buttons are pressed
  if (r==14) toDist <: r;
  while (1) {
    b :> r;                     //constantly send export signal to Distributor
    if (r==13) toDist <: 1;     //1 if SW2 is pressed
    else toDist <: 0;           //0 if not
  }
}


/////////////////////////////////////////////////////////////////////////////////////////
//
// Read Image from PGM file from path infname[] to channel c_out
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataInStream(char infname[], chanend c_out)
{
  int res;
  uchar line[ IMWD ];
  printf( "DataInStream: Start...\n" );

  //Open PGM file
  res = _openinpgm( infname, IMWD, IMHT );
  if( res ) {
    printf( "DataInStream: Error openening %s.\n", infname );
    return;
  }

  //Read image line-by-line and send byte by byte to channel c_out
    for( int y = 0; y < IMHT; y++ ) {
      _readinline( line, IMWD );
      for( int x = 0; x < IMWD; x++ ) {
        c_out <: line[ x ];
        //printf( "-%4.1d ", line[ x ] ); //show image values
      }
      //printf( "\n" );
    }

  //Close PGM image file
  _closeinpgm();
  printf( "DataInStream: Done...\n" );
  return;
}


//Modulo function that works for negative ints.
int MOD(int a, int b) {
    int n = ((((a)%(b))+(b))%(b));
    return n;
}

//////////////////////////////////////////////////////////////
//
//Calculates the number of live neighbours of a specific cell.
//
//////////////////////////////////////////////////////////////
int count(uchar board[IMHT/8 + 2][IMWD/8], int y, int x, int i) {
    int liveCount = 0;
    int W = IMWD/8;
    if (i == 0) {       //Edge case when pixel is first in byte.
        if ((board[y-1][MOD((x-1), W)])&1) liveCount++;
        if (((board[y-1][x])>>7)&1) liveCount++;
        if (((board[y-1][x])>>6)&1) liveCount++;
        if ((board[y][MOD((x-1), W)])&1) liveCount++;
        if (((board[y][x])>>6)&1) liveCount++;
        if ((board[y+1][MOD((x-1), W)])&1) liveCount++;
        if (((board[y+1][x])>>7)&1) liveCount++;
        if (((board[y+1][x])>>6)&1) liveCount++;
    }
    else if (i == 7) {      //Edge case when pixel is last in byte
        if ((board[y-1][x]>>1)&1) liveCount++;
        if ((board[y-1][x])&1) liveCount++;
        if (((board[y-1][(x+1)%W])>>7)&1) liveCount++;
        if ((board[y][x]>>1)&1) liveCount++;
        if (((board[y][(x+1)%W])>>7)&1) liveCount++;
        if (((board[y+1][x])>>1)&1) liveCount++;
        if ((board[y+1][x])&1) liveCount++;
        if (((board[y+1][(x+1)%W])>>7)&1) liveCount++;
    }
    else {      //All other cases
        if ((board[y-1][x]>>(8-i))&1) liveCount++;
        if (((board[y-1][x])>>(7-i))&1) liveCount++;
        if (((board[y-1][x])>>(6-i))&1) liveCount++;
        if ((board[y][x]>>(8-i))&1) liveCount++;
        if (((board[y][x])>>(6-i))&1) liveCount++;
        if ((board[y+1][x]>>(8-i))&1) liveCount++;
        if ((board[y+1][x]>>(7-i))&1) liveCount++;
        if (((board[y+1][x])>>(6-i))&1) liveCount++;
    }
    return liveCount;
}

///////////////////////////////////////////////////////////////////////////////////////
//
// Receives input image matrix from DataInStream. Farms out matrix to worker functions.
// Sends signal to workers when pausing/exporting. Sends image matrix to DataOutStream
// when exporting the image matrix.
//
///////////////////////////////////////////////////////////////////////////////////////
void distributor(chanend c_in, chanend c_out, chanend fromAcc, chanend fromButtons, chanend toLEDs, chanend toWorker[8], chanend exportChan[8]) {
  uchar val, newVal;
  uchar board[IMHT][IMWD/8];
  int round = 0;
  int paused = 0;
  int tilted, exported, alive, workerAlive, sw2;
  unsigned int startTime, prevTime;
  unsigned long time;
  unsigned int currentTime = 0;
  timer t;
  int timeCount = 0;
  unsigned int pattern;

  //Starting up and wait for SW1 button of the xCore-200 Explorer to be pressed.
  printf( "ProcessImage: Start, size = %dx%d\n", IMHT, IMWD );
  printf( "Waiting for SW1 to be pressed...\n" );
  fromButtons :> int value;

  //Read in and distribute to Workers.
  printf( "\nReading input image in...\n" );
  pattern = 4;          //Light Green LED
  toLEDs <: pattern;
  for( int y = 0; y < IMHT; y++ ) {             //go through all lines
    for( int x = 0; x < IMWD/8; x++ ) {         //go through all uchars in line
        board[y][x] = 0;
        for (int i = 0; i < 8; i++) {           //go through all pixels in uchar
            c_in :> val;                        //read the pixel value
            board[y][x] |= val/255 << (7-i);    //pack each pixel as bit in uchar
        }
    }
  }
  //Farming sections of the image matrix to Workers.
  for (int w = 0; w < 8; w++) {
      for (int y = (IMHT/8*w)-1; y < (IMHT/8*w)+(IMHT/8)+1; y++) {
          for (int x = 0; x < IMWD/8; x++) {
              if (y == -1) toWorker[w] <: (uchar)board[IMHT-1][x];      //if line == -1, send last line
              else if (y == IMHT) toWorker[w] <:  (uchar)board[0][x];   //if line is over the end of image matrix, send first line
              else toWorker[w] <:  (uchar)board[y][x];                  //else send current line
          }
      }
  }
  pattern = 0;          //Turn off Green LED when reading finishes.
  toLEDs <: pattern;
  printf("\nProcessing!\n");
  t :> startTime;       //Record time when processing begins.
  while (1) {
      fromAcc :> tilted;                                        //Receive tilt signal from orientation.
      fromButtons :> sw2;                                       //Receive export signal from buttonListener.
      for (int w = 0; w < 8; w++) toWorker[w] <: tilted;        //Send tilt signal to all workers.
      //When pausing:
      if (tilted && !paused) {
          printf("\nPAUSED. DISPLAYING STATUS:\n");
          paused = 1;
          pattern = 8;          //Light Red LED
          toLEDs <: pattern;
          printf("Current Round = %d.\n", round);
          alive = 0;
          for (int w = 0; w < 8; w++) {
              toWorker[w] :> workerAlive;       //Receive number of Live Cells from each worker
              alive = alive + workerAlive;      //Calculate total Live Cells
          }
          printf("Number of Live Cells = %d.\n", alive);
          prevTime = currentTime;
          t :> currentTime;
          if (prevTime  > currentTime) timeCount++;                          //Checks if timer has overflowed and corrects.
          time = (timeCount*4294967295 + currentTime-startTime)/100000;      //Calculate total processing time.
          printf("Processing Time Elapsed = %lu milliseconds.\n", time);
      }
      if (!tilted) {
          pattern = pattern & 1;    //Turn off Red LED
          pattern = pattern ^ 1;    //Flash Green LED
          toLEDs <: pattern;
          round++;                  //Increment Round Counter
          if (paused == 1) {
              printf("\nProcessing!\n");
              paused = 0;
          }
          exported = 0;
      }

      for (int i = 0; i < 8; i++) exportChan[i] <: sw2;         //Send export signal to all workers.
      //When Exporting:
      if (sw2 && !exported) {
          printf("EXPORTING!\n");
          pattern = pattern | 2;        //Light Blue LED
          toLEDs <: pattern;
          exported = 1;
          for (int w = 0; w < 8; w++) {                                 //Receive image matrix sections from all workers.
              for (int y = IMHT/8*w; y < (IMHT/8*w)+(IMHT/8); y++) {
                  for (int x = 0; x < IMWD/8; x++) {
                      exportChan[w] :> val;
                      for (int i = 0; i < 8; i++) {
                          newVal = ((val>>(7-i))&1)*255;                //Unpack each bit into uchars.
                          //printf( "-%4.1d ", newVal ); //show image values
                          c_out <: (uchar)newVal;                       //Send each pixel to DataOutStream
                      }
                  }
                  //printf("\n");
              }
          }
          pattern = pattern & 13;       //Turn off Blue LED
          toLEDs <: pattern;
      }
  }
}

void worker(chanend c1, chanend c2, chanend d, chanend e, int id) {
    uchar current;
    uchar section[IMHT/8 + 2][IMWD/8];
    uchar newSection[IMHT/8][IMWD/8];
    int liveCount, tilted, export;
    int alive = 0;
    int round = 0;
    int paused = 0;
    int exported = 0;

    for (int y = 0; y < IMHT/8 + 2; y++) {
        for (int x = 0; x < IMWD/8; x++) {
            d :> section[y][x];                 //Receive section of image matrix from Distributor.
        }
    }
    while (1) {
        d :> tilted;        //Receive tilt signal from Distributor
        //When paused:
        if (tilted && !paused) {
            paused = 1;
            alive = 0;
            for (int y = 1; y < IMHT/8 + 1; y++) {
                for (int x = 0; x < IMWD/8; x++) {
                    for (int i = 0; i < 8; i++) {
                        if ((section[y][x]>>(7-i))&1) alive++;  //Count Live Cells in section and send to Distributor
                    }
                }
            }
            d <: alive;
        }
        if (!tilted) {
            paused = 0;
            exported = 0;
            round++;
            for (int y = 0; y < IMHT/8; y++) {
                for (int x = 0; x < IMWD/8; x++) {
                    newSection[y][x] = 0;           //Clear contents of newSection
                }
            }
            for( int y = 1; y < IMHT/8 + 1; y++ ) {
                for( int x = 0; x < IMWD/8; x++ ) {
                  for (int i = 0; i < 8; i++) {
                      current = (section[y][x]>>(7-i))&1;                                   //For each pixel in section
                      liveCount = count(section, y, x, i);                                  //Calculates number of live neighbour Cells.
                      if (current == 0 && liveCount == 3) current = 1;                      //Implementing transition rules of Game of Life
                      if (current == 1 && (liveCount < 2 || liveCount > 3)) current = 0;
                      newSection[y-1][x] |= current<<(7-i);                                 //Put updated pixel into newSection
                  }
                }
            }
            if (id%2 == 0) {
                for (int x = 0; x < IMWD/8; x++) {
                    c1 <: (uchar)newSection[0][x];              //Even IDs send edge lines
                    c2 <: (uchar)newSection[IMHT/8 - 1][x];
                }
            }
            if (id%2 == 1) {
                for (int x = 0; x < IMWD/8; x++) {
                    c2 :> section[IMHT/8 + 1][x];               //Odd IDs receive edge lines
                    c1 :> section[0][x];
                }
            }
            if (id%2 == 1) {
                 for (int x = 0; x < IMWD/8; x++) {
                     c1 <: (uchar)newSection[0][x];             //Odd IDs send edge lines
                     c2 <: (uchar)newSection[IMHT/8 - 1][x];
                 }
            }
            if (id%2 == 0) {
                 for (int x = 0; x < IMWD/8; x++) {
                     c2 :> section[IMHT/8 + 1][x];              //Even IDs receive edge lines.
                     c1 :> section[0][x];
                 }
            }
            for( int y = 0; y < IMHT/8; y++ ) {
                for( int x = 0; x < IMWD/8; x++ ) {
                      section[y+1][x] = newSection[y][x];       //Update section with new pixel values.
                }
            }
        }
        e :> export;        //Receive export signal from Distributor
        //When Exporting:
        if (export && !exported) {
            exported = 1;
            for (int y = 1; y < IMHT/8 + 1; y++) {
                for (int x = 0; x < IMWD/8; x++) {
                    e <: (uchar)section[y][x];          //Send all bytes in section to Distributor
                }
            }
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////
//
// Write pixel stream from channel c_in to PGM image file
//
/////////////////////////////////////////////////////////////////////////////////////////
void DataOutStream(char outfname[], chanend c_in)
{
  int res;
  uchar line[ IMWD ];

  //Open PGM file
  printf( "DataOutStream: Start...\n" );
  res = _openoutpgm( outfname, IMWD, IMHT );
  if( res ) {
    printf( "DataOutStream: Error opening %s\n.", outfname );
    return;
  }

  //Compile each line of the image and write the image line-by-line
  while (1) {
      for( int y = 0; y < IMHT; y++ ) {
        for( int x = 0; x < IMWD; x++ ) {
          c_in :> line[ x ];
          //printf( "-%4.1d ", line[ x ] ); //show image values
        }
        _writeoutline( line, IMWD );
        //printf( "DataOutStream: Line written...\n" );
      }

  //Close the PGM image
  _closeoutpgm();
  printf( "DataOutStream: Done...\n" );
  }
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Initialise and read orientation, send tilt events to Distributor to pause processing.
//
/////////////////////////////////////////////////////////////////////////////////////////
void orientation( client interface i2c_master_if i2c, chanend toDist) {
  i2c_regop_res_t result;
  char status_data = 0;
  //int tilted = 0;

  // Configure FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_XYZ_DATA_CFG_REG, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }
  
  // Enable FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_CTRL_REG_1, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }

  //Probe the orientation x-axis forever
  while (1) {

    //check until new orientation data is available
    do {
      status_data = i2c.read_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_DR_STATUS, result);
    } while (!status_data & 0x08);

    //get new x-axis tilt value
    int x = read_acceleration(i2c, FXOS8700EQ_OUT_X_MSB);

    //constantly send signal to distributor
    if (x>30) {
        toDist <: 1;       //send 1 when tilted
    }
    else toDist <: 0;            //send 0 when not
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// Orchestrate concurrent system and start up all threads
//
/////////////////////////////////////////////////////////////////////////////////////////
int main(void) {

i2c_master_if i2c[1];               //interface to orientation

chan c_inIO, c_outIO, c_control, buttonsToDist, ledsToDist, workerChan[8], distChan[8], exportChan[8];

par {
    on tile[0]: i2c_master(i2c, 1, p_scl, p_sda, 10);                                                       //server thread providing orientation data
    on tile[0]: orientation(i2c[0],c_control);                                                              //client thread reading orientation data
    on tile[0]: DataInStream(infname, c_inIO);                                                              //thread to read in a PGM image
    on tile[0]: DataOutStream(outfname, c_outIO);                                                           //thread to write out a PGM image
    on tile[0]: distributor(c_inIO, c_outIO, c_control, buttonsToDist, ledsToDist, distChan, exportChan);   //thread to coordinate work on image
    on tile[0]: buttonListener(buttons, buttonsToDist);                                                     //thread to provide button data
    on tile[0]: showLEDs(leds, ledsToDist);                                                                 //thread to output LED data
    on tile[0]: worker(workerChan[0], workerChan[1], distChan[0], exportChan[0], 0);                        //worker threads to process sections of image
    on tile[1]: worker(workerChan[1], workerChan[2], distChan[1], exportChan[1], 1);
    on tile[1]: worker(workerChan[2], workerChan[3], distChan[2], exportChan[2], 2);
    on tile[1]: worker(workerChan[3], workerChan[4], distChan[3], exportChan[3], 3);
    on tile[1]: worker(workerChan[4], workerChan[5], distChan[4], exportChan[4], 4);
    on tile[1]: worker(workerChan[5], workerChan[6], distChan[5], exportChan[5], 5);
    on tile[1]: worker(workerChan[6], workerChan[7], distChan[6], exportChan[6], 6);
    on tile[1]: worker(workerChan[7], workerChan[0], distChan[7], exportChan[7], 7);
  }

  return 0;
}
