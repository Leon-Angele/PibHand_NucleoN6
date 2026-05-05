/**
 * @file hand_config.hpp
 * @brief Zentrales Konfigurations-Header für die Handsteuerung (6 Servos pro Hand)
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
#define DEBUG_PRINTS 1
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
    Count
};

/**
 * @brief Physische Limits und Parameter eines einzelnen Motors
 * @note Alle Positionen/Offsets sind in nativen SmartServo-Einheiten (0..4095).
 */
struct FingerConfig {
    std::string_view name;
    uint16_t minPos;      // 0 in Servo-Einheiten
    uint16_t maxPos;      // Max (4095) in Servo-Einheiten
    uint16_t maxSpeed;    // Standard: 2000
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
 * @brief Globale Achsen-Konfiguration (identisch für links und rechts)
 */
constexpr std::array<FingerConfig, static_cast<size_t>(Finger::Count)> AxisSettings = {{
    {"Thumb Stretch", 0, 4095, 2000, 1500},
    {"Index Stretch", 0, 4095, 2000, 1500},
    {"Middle Stretch", 0, 4095, 2000, 1500},
    {"Ring Stretch", 0, 4095, 2000, 1500},
    {"Pinky Stretch", 0, 4095, 2000, 1500},
    {"Thumb Opposition", 0, 4095, 2000, 1500}
}};

/**
 * @brief Griff-Datenbank basierend auf nativen SmartServo-Einheiten (0-4095)
 */
constexpr std::array<GripConfig, static_cast<size_t>(GripType::Count)> GripDatabase = {{
    {GripType::Open, "OPEN", {0, 0, 0, 0, 0, 0}},
    {GripType::Spitzgriff, "SPITZGRIFF", {4095, 4095, 4095, 4095, 4095, 4095}},
    {GripType::Dreipunktgriff, "DREIPUNKTGRIFF", {3185, 3185, 3185, 0, 0, 2047}},
    {GripType::Schluesselgriff, "SCHLUESSELGRIFF", {2730, 1365, 0, 0, 0, 2730}},
    {GripType::Zylindergriff, "ZYLINDERGRIFF", {3640, 3640, 3640, 3640, 3640, 1365}},
    {GripType::Hakengriff, "HAKENGRIFF", {0, 3640, 3640, 3640, 3640, 0}},
    {GripType::SphaerischerGriff, "SPHAERISCHER_GRIFF", {2730, 2730, 2730, 2730, 2730, 1820}}
}};

/**
 * @brief Hilfsklasse zur ID-Auflösung
 */
class Hand {
public:
    enum class Side { Left, Right };

    /**
     * @brief Berechnet die Servo-ID basierend auf Handseite und Finger
     * Rechte Hand: 1..6, Linke Hand: 7..12
     */
    static constexpr uint8_t getServoID(Side side, Finger finger) {
        uint8_t baseID = (side == Side::Right) ? 1 : 7;
        return baseID + static_cast<uint8_t>(finger);
    }

    /**
     * @brief Konvertiert Eingabewert in Servo-Position.
     *
     * Nach der Umstellung werden die Griffwerte direkt in nativen
     * Servo-Einheiten (0..4095) gespeichert, daher ist diese Funktion
     * aktuell identisch zur Identität und gibt den Wert unverändert zurück.
     */
    static constexpr uint16_t mapToServoPos(uint16_t servoPos) {
        return servoPos;
    }
};

} // namespace HandControl

#endif // __cplusplus

#endif // HAND_CONFIG_HPP
