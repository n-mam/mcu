#ifndef SERIALPORTMANAGER_H
#define SERIALPORTMANAGER_H

#include <QObject>
#include <QSerialPort>

#include <ServerManager.h>

class SerialPortManager : public ServerManager {

    Q_OBJECT

    public:

    SerialPortManager(QObject *parent = nullptr);

    ~SerialPortManager();

    Q_INVOKABLE void disconnect();
    Q_INVOKABLE void connectToPort(QString port);

    virtual void transportSink(const std::string& data) override;

    private slots:

    void onReadyRead();
    void onErrorOccurred(QSerialPort::SerialPortError error);

    private:

    QSerialPort *_port = nullptr;
};

#endif // SERIALPORTMANAGER_H
