#include <itomp_nlp/interface/itomp_nlp_widget.h>


namespace itomp
{

ItompNLPWidget::ItompNLPWidget(QWidget* parent)
    : QWidget(parent)
{
    layout_ = new QGridLayout(this);

    text_edit_ = new QTextEdit(this);
    connect(text_edit_, SIGNAL(textChanged()), this, SLOT(textChanged()));

    layout_->addWidget(text_edit_, 0, 0);
}

void ItompNLPWidget::textChanged()
{
    std::string s = text_edit_->toPlainText().toStdString();
    if (s[s.length() - 1] != '\n')
        return;
    s = s.substr(0, s.length() - 1);

    size_t idx = s.find_last_of('\n');

    if (idx == std::string::npos)
        emit commandAdded(s);

    else
        emit commandAdded(s.substr(idx + 1));
}

}
