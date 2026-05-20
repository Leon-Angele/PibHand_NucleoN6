/**
 * @file hand_config.hpp
 * @brief Zentrales Konfigurations-Header für die Handsteuerung (6 Servos pro Hand)
 * @author Leon Angele
 * @date 2026-05-08
 */

#ifndef HAND_CONFIG_HPP
#define HAND_CONFIG_HPP

#ifdef __cplusplus
#include <cstdint>
#include <array>
#include <string_view>
#else
#include <stdint.h>
#endif

// Debug print macro: toggle via DEBUG_PRINTS (0 = off, 1 = on)
#ifndef DEBUG_PRINTS
#define DEBUG_PRINTS 0
#endif

#if DEBUG_PRINTS
#ifdef __cplusplus
#include <cstdio>
#define HAND_DEBUG(fmt, ...) std::printf("[HAND] " fmt "\r\n", ##__VA_ARGS__)
#else
#include <stdio.h>
#define HAND_DEBUG(fmt, ...) printf("[HAND] " fmt "\r\n", ##__VA_ARGS__)
#endif
#else
#define HAND_DEBUG(fmt, ...) ((void)0)
#endif

#ifdef __cplusplus
namespace HandControl {

/**
 * @brief Definition der Finger-Indizes innerhalb einer Hand
 *
 * Hinweis: Die Servos haben feste IDs in der Reihe.
 * Rechte Hand: IDs 1..6, Linke Hand: IDs 7..12
 */
enum class Finger : uint8_t {
    Thumb = 0,
    Index = 1,
    Middle = 2,
    Ring = 3,
    Pinky = 4,
    ThumbRotation = 5,
    Count = 6
};

/**
 * @brief Verfügbare Griff-Typen
 */
enum class GripType : uint8_t {
    Open,
    Spitzgriff,
    Dreipunktgriff,
    Schluesselgriff,
    Zylindergriff,
    Hakengriff,
    SphaerischerGriff,
    Mittelfinger,
    Count
};

/**
 * @brief Physische Limits und Parameter eines einzelnen Motors
 * @note Alle Positionen sind in nativen SmartServo-Einheiten (0..4095).
 *       zeroPos: Ruhestellung (meist 2047)
 *       maxPos: Maximal ausgefahrene Position (Richtung wird automatisch erkannt)
 */
struct FingerConfig {
    std::string_view name;
    uint16_t zeroPos;     // Neutral/rest position (Ruhestellung, meist 2047)
    uint16_t maxPos;      // Maximum extended position (kann < oder > zeroPos sein)
    uint16_t maxSpeed;    // in Grad/s (0-4095 = 360°)
    uint16_t maxCurrent;  // in mA
};

/**
 * @brief Definition eines Griffs (Soll-Positionen für alle 6 Motoren)
 */
struct GripConfig {
    GripType type;
    std::string_view name;
    std::array<uint16_t, static_cast<size_t>(Finger::Count)> positions;
};

/**
 * @brief Servo-IDs für die LINKE Hand
 * 
 * Reihenfolge: Thumb, Index, Middle, Ring, Pinky, ThumbRotation
 * Hier eintragen: Deine tatsächlichen Hardware-IDs
 */
constexpr std::array<uint8_t, static_cast<size_t>(Finger::Count)> LeftHandIDs = {{
    31,  // Thumb
    32,  // Index
    33,  // Middle
    34,  // Ring
    35,  // Pinky
    30   // ThumbRotation
}};

/**
 * @brief Servo-IDs für die RECHTE Hand
 * 
 * Reihenfolge: Thumb, Index, Middle, Ring, Pinky, ThumbRotation
 * TODO: Hier die IDs für die rechte Hand eintragen
 */
constexpr std::array<uint8_t, static_cast<size_t>(Finger::Count)> RightHandIDs = {{
    1,  // Thumb (TODO: anpassen)
    2,  // Index (TODO: anpassen)
    3,  // Middle (TODO: anpassen)
    4,  // Ring (TODO: anpassen)
    5,  // Pinky (TODO: anpassen)
    6   // ThumbRotation (TODO: anpassen)
}};

/**
 * @brief Achsen-Konfiguration für LINKE Hand
 *
 * Each entry: { "<name>", <zeroPos>, <maxPos>, <maxSpeed>, <maxCurrent> }
 * - zeroPos: Ruhestellung (offen), meist 2047
 * - maxPos: Geschlossene Position (kann < oder > zeroPos sein)
 * - maxSpeed: Maximale Geschwindigkeit in Grad/s
 * - maxCurrent: Maximaler Strom in mA
 */
constexpr std::array<FingerConfig, static_cast<size_t>(Finger::Count)> LeftAxisSettings = {{
    {"Thumb Stretch", 2047, 0, 200, 200},           // ID 31: bewegt sich von 2047 → 0
    {"Index Stretch", 2047, 4096, 200, 200},        // ID 32: bewegt sich von 2047 → 4096
    {"Middle Stretch", 2047, 0, 200, 200},          // ID 33: bewegt sich von 2047 → 0
    {"Ring Stretch", 2047, 4096, 200, 200},         // ID 34: bewegt sich von 2047 → 4096
    {"Pinky Stretch", 2047, 4096, 200, 200},        // ID 35: bewegt sich von 2047 → 4096
    {"Thumb Opposition", 2047, 500, 200, 200}       // ID 30: bewegt sich von 2047 → 500
}};

/**
 * @brief Achsen-Konfiguration für RECHTE Hand
 * 
 * TODO: Hier die Werte für die rechte Hand anpassen (spiegeln oder separat konfigurieren)
 */
constexpr std::array<FingerConfig, static_cast<size_t>(Finger::Count)> RightAxisSettings = {{
    {"Thumb Stretch", 2047, 0, 200, 200},       
    {"Index Stretch", 2047, 4096, 200, 200},       
    {"Middle Stretch", 2047, 0, 200, 200},      
    {"Ring Stretch", 2047, 4096, 200, 200},        
    {"Pinky Stretch", 2047, 4096, 200, 200},       
    {"Thumb Opposition", 2047, 500, 200, 200}     
}};

/**
 * @brief Griff-Datenbank basierend auf nativen SmartServo-Einheiten (0-4095)
 */
constexpr std::array<GripConfig, static_cast<size_t>(GripType::Count)> GripDatabase = {{
    {GripType::Open, "OPEN", {0, 0, 0, 0, 0, 0}},
    {GripType::Spitzgriff, "ZEIGEN", {4095, 0, 4095, 4095, 4095, 2047}},
    {GripType::Dreipunktgriff, "DREIPUNKTGRIFF", {3185, 3185, 3185, 0, 0, 2047}},
    {GripType::Schluesselgriff, "SCHLUESSELGRIFF", {2730, 1365, 0, 0, 0, 2730}},
    {GripType::Zylindergriff, "ZYLINDERGRIFF", {3640, 3640, 3640, 3640, 3640, 1365}},
    {GripType::Hakengriff, "HAKENGRIFF", {0, 3640, 3640, 3640, 3640, 0}},
    {GripType::SphaerischerGriff, "SPHAERISCHER_GRIFF", {2730, 2730, 2730, 2730, 2730, 1820}}
    ,{GripType::Mittelfinger, "MITTELFINGER", {4095, 4095, 0, 4095, 4095, 2000}}
}};

/**
 * @brief Hilfsklasse zur ID-Auflösung und Positions-Mapping
 */
class Hand {
public:
    enum class Side { Left, Right };

    /**
     * @brief Holt die Servo-ID für einen bestimmten Finger
     * @param side Handseite (Left/Right)
     * @param finger Finger-Index
     * @return Hardware-Servo-ID aus den konfigurierten ID-Arrays
     */
    static constexpr uint8_t getServoID(Side side, Finger finger) {
        const auto& ids = (side == Side::Left) ? LeftHandIDs : RightHandIDs;
        return ids[static_cast<size_t>(finger)];
    }
    
    /**
     * @brief Holt die Achsen-Konfiguration für einen Finger
     * @param side Handseite (Left/Right)
     * @param finger Finger-Index
     * @return Referenz auf FingerConfig
     */
    static constexpr const FingerConfig& getAxisConfig(Side side, Finger finger) {
        const auto& settings = (side == Side::Left) ? LeftAxisSettings : RightAxisSettings;
        return settings[static_cast<size_t>(finger)];
    }

    /**
     * @brief Konvertiert logische Position in physische Servo-Position
     *
     * Logische Position: 0 = offen (Ruhestellung), 4095 = geschlossen (maximal)
     * Physische Position: berücksichtigt zeroPos und Drehrichtung automatisch
     *
     * Formel: servo = zeroPos + (logicalPos * (maxPos - zeroPos) / 4095)
     * Funktioniert für beide Drehrichtungen (positive und negative Delta)
     *
     * @param side Handseite
     * @param finger Finger-Index
     * @param logicalPos Logische Position (0..4095, 0=offen)
     * @return Physische Servo-Position (0..4095)
     */
    static constexpr uint16_t mapToServoPos(Side side, Finger finger, uint16_t logicalPos) {
        const auto& cfg = getAxisConfig(side, finger);
        
        // Lineare Interpolation von zeroPos zu maxPos
        // Funktioniert für beide Richtungen (positiv und negativ)
        int32_t delta = static_cast<int32_t>(cfg.maxPos) - static_cast<int32_t>(cfg.zeroPos);
        int32_t offset = (static_cast<int32_t>(logicalPos) * delta) / 4095;
        int32_t servoPos = static_cast<int32_t>(cfg.zeroPos) + offset;
        
        // Clamp auf gültigen Bereich [0, 4095]
        if (servoPos < 0) servoPos = 0;
        if (servoPos > 4095) servoPos = 4095;
        
        return static_cast<uint16_t>(servoPos);
    }
};

} // namespace HandControl

#endif // __cplusplus

#endif // HAND_CONFIG_HPP
