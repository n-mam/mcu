#ifndef SERVER_H
#define SERVER_H

#include <QObject>

class ServerManager : public QObject {

    Q_OBJECT

    public:

    ServerManager(QObject *parent = nullptr);
    ~ServerManager();

    void logToQml(const std::string&);
    virtual void transportSink(const std::string& data) = 0;

    Q_INVOKABLE void resetMCU();
    Q_INVOKABLE void stopWorkflowLoop();
    Q_INVOKABLE void sendMessage(QString);
    Q_INVOKABLE void sendKeyValue(uint32_t, float);

    Q_PROPERTY(bool decode READ getDecode WRITE setDecode NOTIFY decodeChanged);

    bool getDecode();
    void setDecode(bool);
    QString getLocalIPAddress();
    void processData(QByteArray&);

    signals:

    void decodeChanged();
    void qmlLog(QString);
    void linkStatus(bool);

    private:

    void writeDataA();
    void writeDataB();
    void writeDataC();

    bool m_decode = false;
};

#endif // SERVER_H
