#ifndef TCPSERVERMANAGER_H
#define TCPSERVERMANAGER_H

#include <QObject>
#include <QTcpSocket>

#include <ServerManager.h>

class TcpServerManager : public ServerManager {

    Q_OBJECT

    public:

    TcpServerManager(QObject *parent = nullptr);
    ~TcpServerManager();

    virtual void transportSink(const std::string& data) override;

    Q_INVOKABLE void tcpHostConnect(QString host, int port);

    void onConnected();
    void onReadyRead();
    void onDisconnected();
    void onErrorOccurred(QAbstractSocket::SocketError error);

    private:
    int _port;
    QString _host;
    QTcpSocket _socket;
};

#endif // TcpServerManager_H
