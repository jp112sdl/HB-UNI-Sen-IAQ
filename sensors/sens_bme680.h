//based on https://github.com/G6EJD/BME680-Example

#ifndef _SENS_BME680_H_
#define _SENS_BME680_H_

#include <Wire.h>
#include <Sensors.h>
#include <ClosedCube_BME680.h>


/*
 * Gas is returned as a resistance value in ohms.
 * This value takes up to 30 minutes to stabilize!
 * Once it stabilizes, you can use that as your baseline reading.
 * Higher concentrations of VOC will make the resistance lower.
 */

#define HUM_REFERENCE   40.0
#define HUM_WEIGHTING   0.25     // so hum effect is 25% of the total air quality score
#define GAS_WEIGHTING   0.75     // so gas effect is 75% of the total air quality score
#define GAS_LOWER_LIMIT 50000.0   // Bad air quality limit
#define GAS_UPPER_LIMIT 500000.0  // Good air quality limit
#define AVG_COUNT       1

namespace as {

template <uint8_t ADDRESS=0x77>
class Sens_Bme680 : public Sensor {
private:
  int16_t   _temperature;
  uint16_t  _pressureNN;
  uint8_t   _humidity;
  uint16_t  _iaqPercent;
  uint8_t   _iaqState;

  ClosedCube_BME680 _bme680;
public:

  Sens_Bme680 (): _temperature(0),  _pressureNN(0), _humidity(0), _iaqPercent(0), _iaqState(0) {}
    ~Sens_Bme680 () {}

  void init () {

    Wire.begin();
    DPRINT(F("BME680 "));
    _bme680.init(ADDRESS); // I2C address: 0x76 or 0x77
    if (_bme680.reset() != 0) {
      DPRINTLN(F("ERR"));
      while (1);
    } else DPRINT(F("OK"));
    _present = true;


    DPRINT(", Chip ID=0x");
    DHEXLN(_bme680.getChipID());

      // oversampling: humidity = x1, temperature = x2, pressure = x16
    _bme680.setOversampling(BME680_OVERSAMPLING_X2, BME680_OVERSAMPLING_X2, BME680_OVERSAMPLING_X4);
    _bme680.setIIRFilter(BME680_FILTER_3);
    _bme680.setGasOn(320, 150); // 300 degree Celsius and 100 milliseconds
    _bme680.setForcedMode();
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

      ClosedCube_BME680_Status status = _bme680.readStatus();
      while (! (status.newDataFlag == 1)) {
        _bme680.setForcedMode();
        DPRINT(".");
        _delay_ms(200);
        status = _bme680.readStatus();
      }

      temp = _bme680.readTemperature();
      pres = _bme680.readPressure();
      hum =  _bme680.readHumidity();

      DPRINT("gas: ");
      gas = 0;
      for (uint8_t c = 0; c < AVG_COUNT; c++) {
        while (! (status.newDataFlag == 1)) {
          _bme680.setForcedMode();
          DPRINT(".");
          _delay_ms(200);
          status = _bme680.readStatus();
        }
        gas  += _bme680.readGasResistance();
        status = _bme680.readStatus();
      }
      gas /= AVG_COUNT;
      DDECLN(gas);
      _temperature = (int16_t)(temp * 10);
      _pressureNN  = (uint16_t)(EquivalentSeaLevelPressure(float(height), temp, pres) * 10);
      _humidity    = (uint8_t)hum;

      DPRINT(F("T   = "));DDECLN(_temperature);
      DPRINT(F("P   = "));DDECLN(pres);
      DPRINT(F("PNN = "));DDECLN(_pressureNN);
      DPRINT(F("Hum = "));DDECLN(_humidity);
      DPRINT(F("Gas = "));DDECLN(gas / 1000);

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
      float _hum_score = 0.0;
      float _gas_score = 0.0;
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

        if (gas > GAS_UPPER_LIMIT) gas = GAS_UPPER_LIMIT;
        if (gas < GAS_LOWER_LIMIT) gas = GAS_LOWER_LIMIT;
        //DPRINT(F("gRef= "));DDECLN(_gas_reference);

        _gas_score = (0.75/(GAS_UPPER_LIMIT-GAS_LOWER_LIMIT)*gas -(GAS_LOWER_LIMIT*(0.75/(GAS_UPPER_LIMIT-GAS_LOWER_LIMIT))))*100;

        //Combine results for the final IAQ index value (0-100% where 100% is good quality air)
        float air_quality_score = _hum_score + _gas_score;

        _iaqState = CalculateIAQ(air_quality_score);
        _iaqPercent = (uint8_t)(air_quality_score);
        //DPRINT(F("AQ% = "));DDECLN(_iaqPercent);
        //DPRINT(F("AQS = "));DDECLN(_iaqState);
        DPRINT(F("AQ  = "));DDEC((uint8_t)air_quality_score);DPRINT(F("% (H: "));DDEC((uint8_t)(_hum_score));DPRINT(F("% + G: "));DDEC((uint8_t)(_gas_score));DPRINTLN(F("%)"));
        _bme680.setForcedMode();
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
