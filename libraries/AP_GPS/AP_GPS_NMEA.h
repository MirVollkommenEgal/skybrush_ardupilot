/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// NMEA parser, adapted by Michael Smith from TinyGPS v9:
//
// TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
// Copyright (C) 2008-9 Mikal Hart
// All rights reserved.
//
//

/// @file	AP_GPS_NMEA.h
/// @brief	NMEA protocol parser
///
/// This is a lightweight NMEA parser, derived originally from the
/// TinyGPS parser by Mikal Hart.  It is frugal in its use of memory
/// and tries to avoid unnecessary arithmetic.
///
/// The parser handles GPGGA, GPRMC and GPVTG messages, and attempts to be
/// robust in the face of occasional corruption in the input stream.  It
/// makes a basic effort to configure GPS' that are likely to be connected in
/// NMEA mode (SiRF, MediaTek and ublox) to emit the correct message
/// stream, but does not validate that the correct stream is being received.
/// In particular, a unit emitting just GPRMC will show as having a fix
/// even though no altitude data is being received.
///
/// GPVTG data is parsed, but as the message may not contain the the
/// qualifier field (this is common with e.g. older SiRF units) it is
/// not considered a source of fix-valid information.
///
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_GPS_NMEA_ENABLED
/// NMEA parser
///
class AP_GPS_NMEA : public AP_GPS_Backend
{
    friend class AP_GPS_NMEA_Test;

public:

    using AP_GPS_Backend::AP_GPS_Backend;

    /// Checks the serial receive buffer for characters,
    /// attempts to parse NMEA data and updates internal state
    /// accordingly.
    bool        read() override;

	static bool _detect(struct NMEA_detect_state &state, uint8_t data);

    const char *name() const override { return "NMEA"; }

    // driver specific health, returns true if the driver is healthy
    bool is_healthy(void) const override;

    bool is_configured(void) const override;

    void broadcast_configuration_failure_reason(void) const override;

    // get lag in seconds
    bool get_lag(float &lag_sec) const override;

#if HAL_LOGGING_ENABLED
    void Write_AP_Logger_Log_Startup_messages() const override;
#endif

private:
    /// Coding for the GPS sentences that the parser handles
    enum _sentence_types : uint16_t {      //there are some more than 10 fields in some sentences , thus we have to increase these value.
        _GPS_SENTENCE_RMC = 32,
        _GPS_SENTENCE_GGA = 64,
        _GPS_SENTENCE_VTG = 96,
        _GPS_SENTENCE_HDT = 128,
        _GPS_SENTENCE_THS = 160, // True heading with quality indicator, available on Trimble MB-Two
        _GPS_SENTENCE_KSXT = 170, // extension for Unicore, 21 fields
        _GPS_SENTENCE_AGRICA = 193, // extension for Unicore, 65 fields
        _GPS_SENTENCE_VERSIONA = 270, // extension for Unicore, version, 10 fields
        _GPS_SENTENCE_UNIHEADINGA = 290, // extension for Unicore, uniheadinga, 20 fields
        _GPS_SENTENCE_OTHER = 0
    };

    /// Update the decode state machine with a new character
    ///
    /// @param	c		The next character in the NMEA input stream
    /// @returns		True if processing the character has resulted in
    ///					an update to the GPS state
    ///
    bool                        _decode(char c);

    /// Parses the @p as a NMEA-style decimal number with
    /// up to 3 decimal digits.
    ///
    /// @returns		The value expressed by the string in @p,
    ///					multiplied by 100.
    ///
    static int32_t _parse_decimal_100(const char *p);

    /// Parses the current term as a NMEA-style degrees + minutes
    /// value with up to four decimal digits.
    ///
    /// This gives a theoretical resolution limit of around 1cm.
    ///
    /// @returns		The value expressed by the string in _term,
    ///					multiplied by 1e7.
    ///
    uint32_t    _parse_degrees();

    /// Processes the current term when it has been deemed to be
    /// complete.
    ///
    /// Each GPS message is broken up into terms separated by commas.
    /// Each term is then processed by this function as it is received.
    ///
    /// @returns		True if completing the term has resulted in
    ///					an update to the GPS state.
    bool                        _term_complete();

    /// return true if we have a new set of NMEA messages
    bool _have_new_message(void);

#if AP_GPS_NMEA_UNICORE_ENABLED
    /*
      parse an AGRICA field
     */
    void parse_agrica_field(uint16_t term_number, const char *term);

    // parse VERSIONA field
    void parse_versiona_field(uint16_t term_number, const char *term);

#if GPS_MOVING_BASELINE
    // parse UNIHEADINGA field
    void parse_uniheadinga_field(uint16_t term_number, const char *term);
#endif
#endif

    // Hinzugefügte Member für den Allystar-Binär-Parser
    enum class AllystarParseState {
        IDLE,
        GOT_SYNC1,
        GOT_MSG_CLASS,
        GOT_MSG_ID,
        GOT_LENGTH1,
        GOT_LENGTH2,
        IN_PAYLOAD,
        IN_CHECKSUM1,
        IN_CHECKSUM2
    } _allystar_parse_state = AllystarParseState::IDLE;

    uint8_t _allystar_msg_class;
    uint8_t _allystar_msg_id;
    uint16_t _allystar_payload_length;
    uint16_t _allystar_payload_counter;
    uint8_t _allystar_ck_a;
    uint8_t _allystar_ck_b;
    uint8_t _allystar_sum_a;
    uint8_t _allystar_sum_b;
    static const uint16_t ALLYSTAR_BUFFER_SIZE = 128;
    uint8_t _allystar_buffer[ALLYSTAR_BUFFER_SIZE];

public:
    static constexpr uint8_t ALLYSTAR_NUM_CONFIG_MSGS = 6;

    struct AllystarMsgRate {
        uint8_t msg_class;
        uint8_t msg_id;
        uint8_t rate;
        bool valid;
    };

    struct AllystarPwrctl2 {
        uint8_t mode;
        uint8_t padding;
        uint16_t ontime_ms;
        int32_t fixfreq;
        uint32_t update_period_ms;
        uint32_t tracking_ms;
        bool valid;
    };

    struct AllystarElev {
        float track_mask_rad;
        float navi_mask_rad;
        bool valid;
    };

    struct AllystarNavSat {
        uint32_t enable_mask;
        bool valid;
    };

    struct AllystarSpeedHold {
        uint16_t speed_cms;
        bool valid;
    };

    struct AllystarCarrSmooth {
        int8_t windows;
        bool valid;
    };

    enum class AllystarConfigTarget : uint8_t {
        PWRCTL2,
        MSG_RATE,
        ELEV,
        NAVSAT,
        SPDHOLD,
        CARRSMOOTH,
    };

    enum class AllystarConfigPhase : uint8_t {
        IDLE,
        POLL_MSG,
        WAIT_POLL_MSG,
        SET_MSG,
        WAIT_ACK_MSG,
        VERIFY_MSG,
        WAIT_VERIFY_MSG,
        SAVE_CFG,
        WAIT_ACK_SAVE,
        COMPLETE,
        FAILED
    };

private:
    bool _allystar_binary_packet_complete();
    void _send_allystar_cfg_pwrctl2(const AllystarPwrctl2 &cfg);
    void _send_allystar_poll_cfg_msg(uint8_t msg_class, uint8_t msg_id);
    void _send_allystar_poll_pwrctl2(void);
    void _send_allystar_poll_cfg_elev(void);
    void _send_allystar_cfg_elev(float track_mask_rad, float navi_mask_rad);
    void _send_allystar_poll_cfg_navsat(void);
    void _send_allystar_cfg_navsat(uint32_t enable_mask);
    void _send_allystar_poll_cfg_spdhold(void);
    void _send_allystar_cfg_spdhold(uint16_t speed_cms);
    void _send_allystar_poll_cfg_carrsmooth(void);
    void _send_allystar_cfg_carrsmooth(int8_t windows);
    void _allystar_config_step(uint32_t now_ms);
    void _allystar_reset_config_state(void);
    bool _allystar_pwrctl2_matches_desired(void) const;
    AllystarPwrctl2 _allystar_desired_pwrctl2() const;
    bool _allystar_msg_matches_desired(uint8_t index) const;
    bool _allystar_current_config_matches_desired(void) const;
    uint8_t _allystar_next_dirty_msg(uint8_t start_index) const;
    bool _allystar_elev_matches_desired(void) const;
    bool _allystar_navsat_matches_desired(void) const;
    bool _allystar_spdhold_matches_desired(void) const;
    bool _allystar_carrsmooth_matches_desired(void) const;
    bool _allystar_have_desired_tracking_min_elev(int16_t &deg) const;
    bool _allystar_have_desired_use_min_elev(int16_t &deg) const;
    bool _allystar_have_desired_speed_hold(uint16_t &speed_cms) const;
    bool _allystar_have_desired_carrsmooth(int8_t &windows) const;
    uint32_t _allystar_desired_navsat_mask(void) const;
    uint8_t _allystar_cfg_subid_for_target(void) const;
    void _allystar_advance_after_msg_poll(void);
    void _allystar_advance_after_msg_verify(void);
    bool _allystar_params_changed(void) const;
    void _allystar_update_shadow_params(void);
    void _allystar_sync_min_elev_params_from_device(void);
    void _allystar_sync_gnss_mode_param_from_device(void);
    void _allystar_sync_speed_hold_param_from_device(void);
    void _allystar_sync_carrsmooth_param_from_device(void);
    void _allystar_mark_configured(bool changed);
    void _allystar_fail_config(const char *reason);
    void _allystar_fallback_to_passive(void);
    const char *_allystar_pending_step_name(void) const;


    uint8_t _parity;                                                    ///< NMEA message checksum accumulator
    uint32_t _crc32;                                            ///< CRC for unicore messages
    bool _is_checksum_term;                                     ///< current term is the checksum
    char _term[30];                                                     ///< buffer for the current term within the current sentence
    uint16_t _sentence_type;                                     ///< the sentence type currently being processed
    bool _is_unicore;                                           ///< true if in a unicore '#' sentence
    uint16_t _term_number;                                       ///< term index within the current sentence
    uint8_t _term_offset;                                       ///< character offset with the term being received
    uint16_t _sentence_length;
    bool _sentence_done;                                        ///< set when a sentence has been fully decoded

    // The result of parsing terms within a message is stored temporarily until
    // the message is completely processed and the checksum validated.
    // This avoids the need to buffer the entire message.
    int32_t _new_time;                                                  ///< time parsed from a term
    int32_t _new_date;                                                  ///< date parsed from a term
    int32_t _new_latitude;                                      ///< latitude parsed from a term
    int32_t _new_longitude;                                     ///< longitude parsed from a term
    int32_t _new_altitude;                                      ///< altitude parsed from a term
    int32_t _new_speed;                                                 ///< speed parsed from a term
    int32_t _new_course;                                        ///< course parsed from a term
    float   _new_gps_yaw;                                        ///< yaw parsed from a term
    uint16_t _new_hdop;                                                 ///< HDOP parsed from a term
    uint8_t _new_satellite_count;                       ///< satellite count parsed from a term
    uint8_t _new_quality_indicator;                                     ///< GPS quality indicator parsed from a term

    uint32_t _last_RMC_ms;
    uint32_t _last_GGA_ms;
    uint32_t _last_VTG_ms;
    uint32_t _last_yaw_ms;
    uint32_t _last_vvelocity_ms;
    uint32_t _last_vaccuracy_ms;
    uint32_t _last_3D_velocity_ms;
    uint32_t _last_KSXT_pos_ms;
    uint32_t _last_AGRICA_ms;
    uint32_t _last_fix_ms;

    /// @name	Init strings
    ///			In ::init, an attempt is made to configure the GPS
    ///			unit to send just the messages that we are interested
    ///			in using these strings
    //@{
    static const char _SiRF_init_string[];         ///< init string for SiRF units
    static const char _ublox_init_string[];        ///< init string for ublox units
    //@}

    static const char _initialisation_blob[];

    /*
      The KSXT message is an extension from Unicore that gives 3D velocity and yaw
      example: $KSXT,20211016083433.00,116.31296102,39.95817066,49.4911,223.57,-11.32,330.19,0.024,,1,3,28,27,,,,-0.012,0.021,0.020,,*2D
     */
    struct {
        double fields[21];
    } _ksxt;

#if AP_GPS_NMEA_UNICORE_ENABLED
    /*
      unicore AGRICA message parsing
     */
    struct {
        uint32_t start_byte;
        uint8_t rtk_status;
        uint8_t heading_status;
        Vector3f vel_NED;
        Vector3f vel_stddev;
        double lat, lng;
        float alt;
        uint32_t itow;
        float undulation;
        Vector3f pos_stddev;
    } _agrica;

    // unicore VERSIONA parsing
    struct {
        char type[10];
        char version[20];
        char build_date[13];
    } _versiona;
    bool _have_unicore_versiona;

#if GPS_MOVING_BASELINE
    // unicore UNIHEADINGA parsing
    struct {
        float baseline_length;
        float heading;
        float pitch;
        float heading_sd;
    } _uniheadinga;
#endif
#endif // AP_GPS_NMEA_UNICORE_ENABLED
    bool _expect_agrica;

    // last time we sent type specific config strings
    uint32_t last_config_ms;

    void _send_allystar_cfg_msg(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
    void _send_allystar_cfg_cfg(uint32_t action, uint32_t mask);

    AllystarMsgRate _allystar_msg_rates[ALLYSTAR_NUM_CONFIG_MSGS] {};
    AllystarPwrctl2 _allystar_pwrctl2 {};
    AllystarElev _allystar_elev {};
    AllystarNavSat _allystar_navsat {};
    AllystarSpeedHold _allystar_spdhold {};
    AllystarCarrSmooth _allystar_carrsmooth {};
    AllystarConfigPhase _allystar_config_phase = AllystarConfigPhase::IDLE;
    AllystarConfigTarget _allystar_config_target = AllystarConfigTarget::MSG_RATE;
    uint8_t _allystar_config_index = 0;
    uint8_t _allystar_config_retries = 0;
    uint32_t _allystar_last_action_ms = 0;
    uint32_t _allystar_save_mask = 0;
    bool _allystar_config_dirty = false;
    bool _allystar_status_reported = false;
    bool _allystar_passive_mode = false;
    bool _allystar_saw_binary_rx = false;
    bool _allystar_reported_binary_error = false;
    uint32_t _allystar_last_byte_ms = 0;
    int8_t _allystar_last_configured_gnss_mode = 0;
    uint16_t _allystar_last_configured_rate_ms = 200;
    int16_t _allystar_last_configured_tracking_min_elevation = -100;
    int16_t _allystar_last_configured_use_min_elevation = -100;
    int16_t _allystar_last_configured_speed_hold = -1;
    int8_t _allystar_last_configured_carrsmooth = -2;
    bool _allystar_have_shadow_params = false;
    char _allystar_failure_reason[48] {};

    // send type specific config strings
    void send_config(void);
};

#if AP_GPS_NMEA_UNICORE_ENABLED && !defined(NMEA_UNICORE_SETUP)
// we don't know what port the GPS may be using, so configure all 3. We need to get it sending
// one message to allow the NMEA detector to run
#define NMEA_UNICORE_SETUP "CONFIG COM1 230400 8 n 1\r\nCONFIG COM2 230400 8 n 1\r\nCONFIG COM3 230400 8 n 1\r\nGPGGA 0.2\r\n"
#endif

#endif // AP_GPS_NMEA_ENABLED
