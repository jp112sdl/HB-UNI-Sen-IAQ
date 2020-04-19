// based on https://github.com/G6EJD/BME680-Example
// https://github.com/pimoroni/bme680-python/blob/master/examples/indoor-air-quality.py

#ifndef _SENS_BME680_H_
#define _SENS_BME680_H_

#include <Wire.h>
#include <Sensors.h>
#include <ClosedCube_BME680.h>


/*
 * Gas is returned as a resistance value in ohms.
 * After switch on, it takes up to 30 minutes of measurements in the desired mode to get stable measurement results.
 * Once it stabilizes, you can use that as your baseline reading.
 * Higher concentrations of VOC will make the resistance lower.
 */

#define HUM_REFERENCE   40.0
#define HUM_WEIGHTING   0.00      // so hum effect is 0% of the total air quality score, default is 25%
#define GAS_WEIGHTING   (1.00-(HUM_WEIGHTING))      // so gas effect is 100% of the total air quality score, default is 75%; sum of HUM_WEIGHTING and GAS_WEIGHTING is 1.0
#define HUM_DELTA       5.0       // band around HUM_REFERENCE for best humidity, i.e. 35% .. 45% relative humidity as default
#define GAS_LOWER_LIMIT 2000.0    // Initial setting for bad  air quality lower limit; will automatically adjusted when sensor exposed to a bad smell e.g. parmesan cheese, mustard, clementine or orange peel, disinfectant solution, etc. 
#define GAS_UPPER_LIMIT 150000.0  // Initial setting for good air quality upper limit; will automatically adjusted when sensor is put to outdoor for few hours                
#define AVG_COUNT       5
#define IIR_FILTER_COEFFICIENT 0.9998641  // Decay to 0.71 in about one week for a 4 min sampling period (in 2520 sampling periods)
#define GAS_FACTOR      1.0      // for calclulating the _gas_score the upper gas limit is scaled by this factor in order to get more meaningful results for indoor sensors
                                 // GAS_FACTOR should be set to 1.0 for an outdoor sensor

namespace as {

template <uint8_t ADDRESS=0x76>  // I2C address needs to be be set according to your BME680 sensor breakout in the main sketch HB-UNI-Sen-IAQ.ino: 'Sens_Bme680<0x76>   bme680;' not here!
class Sens_Bme680 : public Sensor {
private:
  int16_t   _temperature;
  uint16_t  _pressureNN;
  uint8_t   _humidity;
  uint16_t  _iaqPercent;
  uint8_t   _iaqState;
  uint32_t  _max_gas_resistance;  // maximum measured gas resistance
  uint32_t  _min_gas_resistance;  // minimum measured gas resistance
  uint32_t  _gas_lower_limit;     // adaptive lower resistance level for bad air quality
  uint32_t  _gas_upper_limit;     // adaptive upper resistance level for good air quality

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

      // oversampling: humidity = x2, temperature = x8, pressure = x4
    _bme680.setOversampling(BME680_OVERSAMPLING_X2, BME680_OVERSAMPLING_X8, BME680_OVERSAMPLING_X4);
    _bme680.setIIRFilter(BME680_FILTER_3); // supresses spikes 
    _bme680.setGasOn(310, 300); // 310 degree Celsius and 300 milliseconds; please check in debug mode whether '-> Gas heat_stab_r   = 1' is achieved. If '-> Gas heat_stab_r   = 0' then the heating time is to short or the temp target too high
    _bme680.setForcedMode();
    
    _max_gas_resistance = 0;
    _min_gas_resistance = 1000000;
    _gas_upper_limit = GAS_UPPER_LIMIT;
    _gas_lower_limit = GAS_LOWER_LIMIT;
    
  }

  double EquivalentSeaLevelPressure(float altitude, float temp, double pres) {
      double seaPress = NAN;
      if(!isnan(altitude) && !isnan(temp) && !isnan(pres))
          seaPress = (pres / pow(1 - ((0.0065 *(double)altitude) / ((double)temp + (0.0065 *(double)altitude) + 273.15)), 5.257));
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
      double temp(NAN), hum(NAN), pres(NAN); // use type double in order to match the return type of closed cubes's library function readPressure
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
      
      //initial trigger of measurement at the beginning of the averaging loop
      _bme680.setForcedMode();
      _delay_ms(200);
      status = _bme680.readStatus();
     
      gas = 0;
      for (uint8_t c = 0; c < AVG_COUNT; c++) {
        while (! (status.newDataFlag == 1)) {
          _bme680.setForcedMode();
          DPRINT(".");
          _delay_ms(100);
          status = _bme680.readStatus();
        }
        // need to check whether first reading belongs to previous quintuple reading (last reading). Measurement results are indicating this. 
        ClosedCube_BME680_gas_r_lsb gas_status = _bme680.read_gas_r_lsb();
        DPRINT(F("Gas heat_stab_r   = "));DDECLN(gas_status.heat_stab_r);
        DPRINT(F("Gas gas_valid_r   = "));DDECLN(gas_status.gas_valid_r);
        uint32_t _g = _bme680.readGasResistance();
        DPRINT("index: ");DDECLN(c);
        DPRINT("gas: ");DDECLN(_g);
        //DDEC(_g);
        gas  += _g;
        status = _bme680.readStatus();
      }
      
      gas /= AVG_COUNT;
      DPRINT("avg gas: ");DDECLN(gas);
      
      
      
      if ( gas > _max_gas_resistance)    // capture maximum of measured gas resistances
        _max_gas_resistance = gas;
      if ( gas < _min_gas_resistance)    // capture minimum of measured gas resistances
        _min_gas_resistance = gas;
      
      //peak detector for _gas_upper_limit 
      if ( gas > _gas_upper_limit )
      {
        _gas_upper_limit = gas;
      }
      else
      {
        _gas_upper_limit = _gas_upper_limit * IIR_FILTER_COEFFICIENT; // decay each sample by IIR_FILTER_COEFFICIENT
        if ( _gas_upper_limit < GAS_UPPER_LIMIT )
          _gas_upper_limit = GAS_UPPER_LIMIT; // lower limit for _gas_upper_limit
      }
      
      //peak detector for _gas_lower_limit 
      if ( gas < _gas_lower_limit )
      {
        _gas_lower_limit = gas;
      }
      else
      {
        _gas_lower_limit = _gas_lower_limit / IIR_FILTER_COEFFICIENT; // increase each sample by 1.0/IIR_FILTER_COEFFICIENT
        if ( _gas_lower_limit > GAS_LOWER_LIMIT )
          _gas_lower_limit = GAS_LOWER_LIMIT; // upper limit for _gas_lower_limit
      }
      
      _temperature = (int16_t)(temp * 10);
      _pressureNN  = (uint16_t)(EquivalentSeaLevelPressure(float(height), temp, pres)*10.0); 
      _humidity    = (uint8_t)hum;
      
      DPRINT(F("Gas UPPER LIMIT   = "));DDECLN(_gas_upper_limit);
      DPRINT(F("Gas LOWER LIMIT   = "));DDECLN(_gas_lower_limit);

      DPRINT(F("T   = "));DDECLN(_temperature);
      DPRINT(F("P   = "));DDECLN(pres);
      DPRINT(F("PNN = "));DDECLN(_pressureNN);
      DPRINT(F("Hum = "));DDECLN(_humidity);
      DPRINT(F("Gas = "));DDECLN(gas);
      DPRINT(F("Gas FACTOR = "));DDECLN(GAS_FACTOR);

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
      if (hum >= HUM_REFERENCE - HUM_DELTA && hum <= HUM_REFERENCE + HUM_DELTA)
        _hum_score = HUM_WEIGHTING*100.0; // Humidity +/-HUM_DELTA% around optimum HUM_REFERENCE; region 0
      else
      { //sub-optimal, trapezoid function: 0 @ hum=0; HUM_WEIGHTING @ hum = HUM_REFERENCE - HUM_DELTA; HUM_WEIGHTING @ hum = HUM_REFERENCE + HUM_DELTA; 0 for hum >= HUM_REFERENCE + HUM_DELTA;
        if (hum < HUM_REFERENCE - HUM_DELTA)
          _hum_score = HUM_WEIGHTING/(HUM_REFERENCE - HUM_DELTA)*hum*100.0; // region 1; zero for hum = 0, continuous at hum = HUM_REFERENCE - HUM_DELTA
        else // (hum > HUM_REFERENCE + HUM_DELTA)
        {
          if ( hum >= 2 * HUM_REFERENCE )
            _hum_score = 0.0;
          else
          {
            _hum_score = (-HUM_WEIGHTING/(HUM_REFERENCE - HUM_DELTA)*(hum-2*HUM_REFERENCE))*100.0; // region 2; negative slope of region 1; zero for hum = 2 * HUM_REFERENCE, continuous at hum = HUM_REFERENCE + HUM_DELTA
          }
        }
      }
      
      // limit gas values
      if (gas > _gas_upper_limit) gas = _gas_upper_limit;
      if (gas < _gas_lower_limit) gas = _gas_lower_limit;
        
      //DPRINT(F("gRef= "));DDECLN(_gas_reference);

      _gas_score = ( GAS_WEIGHTING/(_gas_upper_limit*GAS_FACTOR-_gas_lower_limit)*gas - GAS_WEIGHTING*_gas_lower_limit/(_gas_upper_limit*GAS_FACTOR-_gas_lower_limit))*100.0;
      
      if ( _gas_score > 100.0 )
        _gas_score = 100.0;

        
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
