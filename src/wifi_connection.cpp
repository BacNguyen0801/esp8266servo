#include "wifi_connection.h"

#include <LittleFS.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

#ifndef WIFI_DEFAULT_SSID
#define WIFI_DEFAULT_SSID "Bap Family"
#endif

#ifndef WIFI_DEFAULT_PASSWORD
#define WIFI_DEFAULT_PASSWORD "0902630801"
#endif

namespace
{
constexpr char WIFI_CONFIG_PATH[] = "/wifi.cfg";
constexpr char WIFI_CONFIG_TEMP_PATH[] = "/wifi.tmp";
constexpr uint8_t WIFI_CONFIG_HEADER_SIZE = 7;
constexpr uint8_t WIFI_MAX_SSID_LENGTH = 32;
constexpr uint8_t WIFI_MAX_PASSWORD_LENGTH = 63;

bool IsValidPassword(const String &password)
{
    return password.length() == 0 ||
        (password.length() >= 8 && password.length() <= WIFI_MAX_PASSWORD_LENGTH);
}
}

bool WifiConnection::begin()
{
    if (!LittleFS.begin())
    {
        Serial.println("[WIFI] Failed to mount LittleFS");
        return false;
    }

    m_storageReady = true;
    WiFi.persistent(false);

    if (!LittleFS.exists(WIFI_CONFIG_PATH))
    {
        const String defaultSsid = WIFI_DEFAULT_SSID;
        const String defaultPassword = WIFI_DEFAULT_PASSWORD;
        if (defaultSsid.length() == 0)
            return true;

        if (defaultSsid.length() > WIFI_MAX_SSID_LENGTH ||
            !IsValidPassword(defaultPassword))
        {
            Serial.println("[WIFI] Default credentials are invalid");
            return false;
        }

        m_ssid = defaultSsid;
        m_password = defaultPassword;
        if (!saveCredentials())
        {
            m_ssid = "";
            m_password = "";
            return false;
        }

        Serial.println("[WIFI] Default credentials saved");
        return true;
    }

    return loadCredentials();
}

bool WifiConnection::setSsid(const String &ssid)
{
    if (!isStorageReady() || ssid.length() > WIFI_MAX_SSID_LENGTH)
    {
        Serial.println("[WIFI] Cannot set SSID: storage unavailable or SSID too long");
        return false;
    }

    if (m_ssid == ssid)
        return true;

    const String previousSsid = m_ssid;
    m_ssid = ssid;
    if (!saveCredentials())
    {
        m_ssid = previousSsid;
        return false;
    }

    m_credentialsChanged = true;
    return true;
}

String WifiConnection::getSsid() const
{
    return m_ssid;
}

bool WifiConnection::setPassword(const String &password)
{
    if (!isStorageReady() || !IsValidPassword(password))
    {
        Serial.println("[WIFI] Cannot set password: storage unavailable or password length is invalid");
        return false;
    }

    if (m_password == password)
        return true;

    const String previousPassword = m_password;
    m_password = password;
    if (!saveCredentials())
    {
        m_password = previousPassword;
        return false;
    }

    m_credentialsChanged = true;
    return true;
}

String WifiConnection::getPassword() const
{
    return m_password;
}

bool WifiConnection::connect()
{
    if (!isStorageReady() || m_ssid.length() == 0 || !IsValidPassword(m_password))
    {
        Serial.println("[WIFI] Cannot connect: storage or credentials are invalid");
        return false;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(m_ssid.c_str(), m_password.c_str());
    m_credentialsChanged = false;
    Serial.printf("[WIFI] Connecting to SSID: %s\n", m_ssid.c_str());
    return true;
}

void WifiConnection::disconnect()
{
    WiFi.disconnect();
}

bool WifiConnection::reconnect()
{
    if (m_ssid.length() == 0)
    {
        Serial.println("[WIFI] Cannot reconnect: SSID is empty");
        return false;
    }

    WiFi.disconnect();
    return connect();
}

bool WifiConnection::isConnected() const
{
    return WiFi.status() == WL_CONNECTED;
}

bool WifiConnection::isChanged() const
{
    return m_credentialsChanged;
}

bool WifiConnection::loadCredentials()
{
    File file = LittleFS.open(WIFI_CONFIG_PATH, "r");
    if (!file)
    {
        Serial.println("[WIFI] Failed to open credential file");
        return false;
    }

    uint8_t header[WIFI_CONFIG_HEADER_SIZE];
    if (file.read(header, sizeof(header)) != sizeof(header) ||
        header[0] != 'W' || header[1] != 'I' || header[2] != 'F' ||
        header[3] != 'I' || header[4] != '1')
    {
        file.close();
        Serial.println("[WIFI] Credential file has an invalid header");
        return false;
    }

    const uint8_t ssidLength = header[5];
    const uint8_t passwordLength = header[6];
    const uint16_t credentialsLength = static_cast<uint16_t>(ssidLength) + passwordLength;
    if (ssidLength > WIFI_MAX_SSID_LENGTH ||
        passwordLength > WIFI_MAX_PASSWORD_LENGTH ||
        file.size() != WIFI_CONFIG_HEADER_SIZE + credentialsLength)
    {
        file.close();
        Serial.println("[WIFI] Credential file has invalid lengths");
        return false;
    }

    uint8_t credentials[WIFI_MAX_SSID_LENGTH + WIFI_MAX_PASSWORD_LENGTH];
    if (file.read(credentials, credentialsLength) != credentialsLength)
    {
        file.close();
        Serial.println("[WIFI] Failed to read credentials");
        return false;
    }
    file.close();

    String ssid;
    String password;
    ssid.reserve(ssidLength);
    password.reserve(passwordLength);
    for (uint8_t i = 0; i < ssidLength; ++i)
        ssid += static_cast<char>(credentials[i]);
    for (uint8_t i = 0; i < passwordLength; ++i)
        password += static_cast<char>(credentials[ssidLength + i]);

    if (!IsValidPassword(password))
    {
        Serial.println("[WIFI] Credential file contains an invalid password");
        return false;
    }

    m_ssid = ssid;
    m_password = password;
    m_credentialsChanged = false;
    return true;
}

bool WifiConnection::saveCredentials()
{
    File file = LittleFS.open(WIFI_CONFIG_TEMP_PATH, "w");
    if (!file)
    {
        Serial.println("[WIFI] Failed to open temporary credential file");
        return false;
    }

    const uint8_t header[WIFI_CONFIG_HEADER_SIZE] = {
        'W', 'I', 'F', 'I', '1',
        static_cast<uint8_t>(m_ssid.length()),
        static_cast<uint8_t>(m_password.length())
    };

    bool written = file.write(header, sizeof(header)) == sizeof(header);
    if (written && m_ssid.length() > 0)
        written = file.write(
            reinterpret_cast<const uint8_t *>(m_ssid.c_str()),
            m_ssid.length()) == m_ssid.length();
    if (written && m_password.length() > 0)
        written = file.write(
            reinterpret_cast<const uint8_t *>(m_password.c_str()),
            m_password.length()) == m_password.length();
    file.close();

    if (!written)
    {
        LittleFS.remove(WIFI_CONFIG_TEMP_PATH);
        Serial.println("[WIFI] Failed to write credentials");
        return false;
    }

    if (!LittleFS.rename(WIFI_CONFIG_TEMP_PATH, WIFI_CONFIG_PATH))
    {
        LittleFS.remove(WIFI_CONFIG_TEMP_PATH);
        Serial.println("[WIFI] Failed to replace credential file");
        return false;
    }

    return true;
}

bool WifiConnection::isStorageReady() const
{
    return m_storageReady;
}
