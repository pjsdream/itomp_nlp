#include <itomp_nlp/interface/itomp_nlp_widget.h>


namespace itomp
{

ItompNLPWidget::ItompNLPWidget(QWidget* parent)
    : QWidget(parent)
    , speech_subscriber_(0)
{
    layout_ = new QGridLayout(this);
    setLayout(layout_);

    const int num_lines = 5;

    text_edit_ = new QTextEdit(this);
    text_edit_->setMaximumHeight( QFontMetrics(text_edit_->font()).lineSpacing() * num_lines );
    connect(text_edit_, SIGNAL(textChanged()), this, SLOT(textChanged()));

    idle_timer_ = new QTimer(this);
    idle_timer_->setInterval(50);
    connect(idle_timer_, SIGNAL(timeout()), this, SLOT(idle()));
    idle_timer_->start();

    layout_->addWidget(text_edit_, 0, 0);
}

void ItompNLPWidget::setSpeechIPAddress(const std::string& ip)
{
    if (speech_subscriber_ != 0)
        delete speech_subscriber_;

    speech_subscriber_ = new speech_network::SpeechSubscriber(ip);
}

void ItompNLPWidget::textChanged()
{
    std::string s = text_edit_->toPlainText().toStdString();
    if (s[s.length() - 1] != '\n')
        return;
    s = s.substr(0, s.length() - 1);

    size_t idx = s.find_last_of('\n');

    if (idx == std::string::npos)
        Q_EMIT commandAdded(s);

    else
        Q_EMIT commandAdded(s.substr(idx + 1));
}

void ItompNLPWidget::idle()
{
    if (speech_subscriber_ != 0)
    {
        std::string string = speech_subscriber_->receive();

        if (string != "")
        {
            // interface string change
            text_edit_->setPlainText(QString((string + "\n").c_str()));
        }
    }
}

}
