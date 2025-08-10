#ifndef UDPSERVERMANAGER_H
#define UDPSERVERMANAGER_H

#include <QObject>
#include <QUdpSocket>

#include <ServerManager.h>

class UdpServerManager : public ServerManager {

    Q_OBJECT

    public:

    UdpServerManager(QObject *parent = nullptr);
    ~UdpServerManager();

    virtual void transportSink(const std::string& data) override;

    public slots:

    void processPendingDatagrams();
    void unicastMessage(const std::string& data, const std::string& ip, quint16 port);

    private:

    QUdpSocket *udpSocket;
};

#endif
