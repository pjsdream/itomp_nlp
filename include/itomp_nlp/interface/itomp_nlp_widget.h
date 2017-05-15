#ifndef ITOMP_INTERFACE_ITOMP_NLP_WIDGET_H
#define ITOMP_INTERFACE_ITOMP_NLP_WIDGET_H


#include <QWidget>
#include <QGridLayout>
#include <QTextEdit>
#include <QTimer>

#include <itomp_nlp/optimization/optimizer.h>

#include <itomp_nlp/network/speech_subscriber.h>


namespace itomp
{

class ItompNLPWidget : public QWidget
{
    Q_OBJECT

public:

    ItompNLPWidget(QWidget* parent = 0);

    void setSpeechIPAddress(const std::string& ip);

Q_SIGNALS:

    void commandAdded(std::string);

private Q_SLOTS:

    void textChanged();
    void idle();

private:

    QGridLayout* layout_;
    QTextEdit* text_edit_;
    QTimer* idle_timer_;

    speech_network::SpeechSubscriber* speech_subscriber_;
};

}


#endif // ITOMP_INTERFACE_ITOMP_NLP_WIDGET_H
