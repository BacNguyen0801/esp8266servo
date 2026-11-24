#pragma once

#include <Arduino.h>

class WifiConnection
{
public:
    bool begin();

    bool setSsid(const String &ssid);
    String getSsid() const;

    bool setPassword(const String &password);
    String getPassword() const;

    bool connect();
    void disconnect();
    bool reconnect();
    bool isConnected() const;
    bool isChanged() const;

private:
    bool loadCredentials();
    bool saveCredentials();
    bool isStorageReady() const;

    String m_ssid;
    String m_password;
    bool m_storageReady = false;
    bool m_credentialsChanged = false;
};
