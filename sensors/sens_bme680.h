//based on https://github.com/G6EJD/BME680-Example

#ifndef _SENS_BME680_H_
#define _SENS_BME680_H_

#include <Wire.h>
#include <Sensors.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

/*
 * Gas is returned as a resistance value in ohms.
 * This value takes up to 30 minutes to stabilize!
 * Once it stabilizes, you can use that as your baseline reading.
 * Higher concentrations of VOC will make the resistance lower.
 */

#define HUM_REFERENCE   40.0
#define HUM_WEIGHTING   0.25     // so hum effect is 25% of the total air quality score
#define GAS_WEIGHTING   0.75     // so gas effect is 75% of the total air quality score
#define GAS_LOWER_LIMIT 5000.0   // Bad air quality limit
#define GAS_UPPER_LIMIT 50000.0  // Good air quality limit

namespace as {

class Sens_Bme680 : public Sensor {
private:
  int16_t   _temperature;
  uint16_t  _pressureNN;
  uint8_t   _humidity;
  uint16_t  _iaqPercent;
  uint8_t   _iaqState;
  uint8_t   _getgasreference_count;
  float     _gas_reference;
  float     _hum_score;
  float     _gas_score;

  // Default : forced mode, standby time = 1000 ms, Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off
  Adafruit_BME680 _bme680;
public:



  Sens_Bme680 (): _temperature(0),  _pressureNN(0), _humidity(0), _iaqPercent(0), _iaqState(0), _getgasreference_count(4), _gas_reference(250000.0), _hum_score(0.0), _gas_score(0.0) {}
    ~Sens_Bme680 () {}


  void GetGasReference(uint8_t readings){
    // Now run the sensor for a burn-in period, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.
    //DPRINTLN(F("Getting a new gas reference value"));
    _getgasreference_count = 0;
    for (int i = 0; i < readings; i++){ // read gas for 10 x 0.150mS = 1.5secs
      uint32_t g = _bme680.readGas();
      //DDEC(g);DPRINT(" ");
      _gas_reference += g;
    }
    DPRINTLN("");
    _gas_reference = _gas_reference / readings;
  }


  void init () {

    Wire.begin();
    DPRINT(F("BME680 "));
    if (!_bme680.begin()) {
      DPRINTLN(F("ERR"));
      while (1);
    } else DPRINTLN(F("OK"));
    _present = true;

    _bme680.setTemperatureOversampling(BME680_OS_2X);
    _bme680.setHumidityOversampling(BME680_OS_2X);
    _bme680.setPressureOversampling(BME680_OS_2X);
    _bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    _bme680.setGasHeater(320, 150); // 320°C for 150 ms
    //an initial reading is needed
    GetGasReference(1);
  }

  float EquivalentSeaLevelPressure(float altitude, float temp, float pres) {
      float seaPress = NAN;
      if(!isnan(altitude) && !isnan(temp) && !isnan(pres))
          seaPress = (pres / pow(1 - ((0.0065 *altitude) / (temp + (0.0065 *altitude) + 273.15)), 5.257));
      return seaPress;
  }

  uint8_t CalculateIAQ(float score){
    enum _eIaqStates:uint8_t {GOOD = 0, AVERAGE = 32, LITTLE_BAD = 64, BAD = 100, WORSE = 196, VERY_BAD = 200};
    uint8_t val = 0;
    score = (100-score)*5;
    if      (score >= 301)                  val= VERY_BAD;
    else if (score >= 201 && score <= 300 ) val= WORSE;
    else if (score >= 176 && score <= 200 ) val= BAD;
    else if (score >= 151 && score <= 175 ) val= LITTLE_BAD;
    else if (score >=  51 && score <= 150 ) val= AVERAGE;
    else if (score >=  00 && score <=  50 ) val= GOOD;
    return val;
  }


  void measure (uint16_t height) {
    if (_present == true) {
      float temp(NAN), hum(NAN), pres(NAN);
      uint32_t gas = 0;

      if (!_bme680.performReading()) {
        DPRINT(F("BME680 read err"));
        return;
      }
      temp = _bme680.temperature;
      hum  = _bme680.humidity;
      pres = _bme680.pressure / 100.0;
      gas  = _bme680.gas_resistance;

      _temperature = (int16_t)(temp * 10);
      _pressureNN  = (uint16_t)(EquivalentSeaLevelPressure(float(height), temp, pres) * 10);
      _humidity    = (uint8_t)hum;

      //DPRINT(F("T   = "));DDECLN(_temperature);
      //DPRINT(F("P   = "));DDECLN(_pressure);
      //DPRINT(F("PNN = "));DDECLN(_pressureNN);
      DPRINT(F("Hum = "));DDECLN(_humidity);
      DPRINT(F("Gas = "));DDECLN(gas);


      /*
       This software, the ideas and concepts is Copyright (c) David Bird 2018. All rights to this software are reserved.

       Any redistribution or reproduction of any part or all of the contents in any form is prohibited other than the following:
       1. You may print or download to a local hard disk extracts for your personal and non-commercial use only.
       2. You may copy the content to individual third parties for their personal use, but only if you acknowledge the author David Bird as the source of the material.
       3. You may not, except with my express written permission, distribute or commercially exploit the content.
       4. You may not transmit it or store it in any other website or other form of electronic retrieval system for commercial purposes.

       The above copyright ('as annotated') notice and this permission notice shall be included in all copies or substantial portions of the Software and where the
       software use is visible to an end-user.

       THE SOFTWARE IS PROVIDED "AS IS" FOR PRIVATE USE ONLY, IT IS NOT FOR COMMERCIAL USE IN WHOLE OR PART OR CONCEPT. FOR PERSONAL USE IT IS SUPPLIED WITHOUT WARRANTY
       OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
       IN NO EVENT SHALL THE AUTHOR OR COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
       FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
       See more at http://www.dsbird.org.uk
      */
        //Calculate humidity contribution to IAQ index
        if (hum >= 38 && hum <= 42)
          _hum_score = 0.25*100; // Humidity +/-5% around optimum
        else
        { //sub-optimal
          if (hum < 38)
            _hum_score = 0.25/HUM_REFERENCE*hum*100;
          else
          {
            _hum_score = ((-0.25/(100-HUM_REFERENCE)*hum)+0.416666)*100;
          }
        }

        //nach 5x measure() eine neue Gas-Referenz aus 10 Gas-Messungen (Mittelwert holen)
        if (_getgasreference_count++ == 5) GetGasReference(10);
        if (_gas_reference > GAS_UPPER_LIMIT) _gas_reference = GAS_UPPER_LIMIT;
        if (_gas_reference < GAS_LOWER_LIMIT) _gas_reference = GAS_LOWER_LIMIT;
        DPRINT(F("gRef= "));DDECLN(_gas_reference);

        _gas_score = (0.75/(GAS_UPPER_LIMIT-GAS_LOWER_LIMIT)*_gas_reference -(GAS_LOWER_LIMIT*(0.75/(GAS_UPPER_LIMIT-GAS_LOWER_LIMIT))))*100;

        //Combine results for the final IAQ index value (0-100% where 100% is good quality air)
        float air_quality_score = _hum_score + _gas_score;

        _iaqState = CalculateIAQ(air_quality_score);
        _iaqPercent = (uint8_t)(air_quality_score);
        //DPRINT(F("AQ% = "));DDECLN(_iaqPercent);
        //DPRINT(F("AQS = "));DDECLN(_iaqState);
        DPRINT(F("AQ  = "));DDEC((uint8_t)air_quality_score);DPRINT(F("% (H: "));DDEC((uint8_t)(_hum_score));DPRINT(F("% + G: "));DDEC((uint8_t)(_gas_score));DPRINTLN(F("%)"));
    }
  }
  
  int16_t  temperature () { return _temperature; }
  uint16_t pressureNN ()  { return _pressureNN; }
  uint8_t  humidity ()    { return _humidity; }
  uint8_t  iaq_percent () { return _iaqPercent; }
  uint8_t  iaq_state ()   { return _iaqState; }
};

}

#endif
