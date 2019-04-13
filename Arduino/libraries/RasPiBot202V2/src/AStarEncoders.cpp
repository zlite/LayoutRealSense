/*
AStarEncoders.cpp
Source: https://github.com/DrGFreeman/RasPiBot202V2

MIT License

Copyright (c) 2017 Julien de la Bruere-Terreault <drgfreeman@tuta.io>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#include <Encoder.h>
#include <AStarEncoders.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder Left(1, 2);
Encoder Right(7, 0);

long positionLeft  = -999;
long positionRight = -999;

volatile uint16_t leftCount;
volatile uint16_t rightCount;



// Constructor
AStarEncoders::AStarEncoders()
{
  Serial.println("Reading encoders");
  leftCount = Left.read();
  rightCount = Right.read();
  if (leftCount != positionLeft || rightCount != positionRight) {
    Serial.print("Left = ");
    Serial.print(leftCount);
    Serial.print(", Right = ");
    Serial.print(rightCount);
    Serial.println();
    positionLeft = leftCount;
    positionRight = rightCount;
	}
  flipDirection(false, false);
}

// Flip count directions
void AStarEncoders::flipDirection(bool left, bool right)
{
  if (left)
  {
    // Flip left encoder directions
    _flipLeft = true;
  }
  else
  {
    _flipLeft = false;
  }
  if (right)
  {
    // Flip right encoder directions
    _flipRight = true;
  }
  else
  {
    _flipRight = false;
  }
}

// Get left counts
int AStarEncoders::getCountsLeft()
{
  cli();
  leftCount = Left.read();
  int counts = leftCount;
  sei();

  if (_flipLeft)
  {
    counts *= -1;
  }

  return counts;
}

// Get right counts
int AStarEncoders::getCountsRight()
{
  cli();
  rightCount = Right.read();
  int counts = rightCount;
  sei();

  if (_flipRight)
  {
    counts *= -1;
  }

  return counts;
}

// Get left counts and reset left counts
int AStarEncoders::getCountsLeftAndReset()
{
  cli();
  int counts = leftCount;
  leftCount = 0;
  sei();

  if (_flipLeft)
  {
    counts *= -1;
  }

  return counts;
}

// Get right counts and reset right counts
int AStarEncoders::getCountsRightAndReset()
{
  cli();
  int counts = rightCount;
  rightCount = 0;
  sei();

  if (_flipRight)
  {
    counts *= -1;
  }

  return counts;
}
