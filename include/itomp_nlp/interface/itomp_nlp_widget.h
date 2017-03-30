#ifndef ITOMP_INTERFACE_ITOMP_NLP_WIDGET_H
#define ITOMP_INTERFACE_ITOMP_NLP_WIDGET_H


#include <QWidget>
#include <QGridLayout>
#include <QTextEdit>

#include <itomp_nlp/optimization/optimizer.h>


namespace itomp
{

class ItompNLPWidget : public QWidget
{
    Q_OBJECT

public:

    ItompNLPWidget(QWidget* parent = 0);

Q_SIGNALS:

    void commandAdded(std::string);

private Q_SLOTS:

    void textChanged();

private:

    QGridLayout* layout_;
    QTextEdit* text_edit_;
};

}


#endif // ITOMP_INTERFACE_ITOMP_NLP_WIDGET_H