/* this code is taken from Qt examples */

#include "config.h"
#include "codeeditor.h"
#include "project.h"
#include "qapplication.h"
#include <QPainter>
#include <QTextObject>
#include "mainwindow.h"
#ifdef KF5_FOUND
  #include <KF5/KSyntaxHighlighting/KSyntaxHighlighting/SyntaxHighlighter>
  #include <KF5/KSyntaxHighlighting/KSyntaxHighlighting/Repository>
  #include <KF5/KSyntaxHighlighting/KSyntaxHighlighting/Definition>
  #include <KF5/KSyntaxHighlighting/KSyntaxHighlighting/Theme>
#endif

using namespace std;

namespace MBSimGUI {

extern MainWindow *mw;

CodeEditor::CodeEditor(QWidget *parent) : QPlainTextEdit(parent) {
  lineNumberArea = new LineNumberArea(this);

  connect(this, &CodeEditor::blockCountChanged, this, &CodeEditor::updateLineNumberAreaWidth);
  connect(this, &CodeEditor::updateRequest, this, &CodeEditor::updateLineNumberArea);

  updateLineNumberAreaWidth(0);

  static const QFont fixedFont=QFontDatabase::systemFont(QFontDatabase::FixedFont);
  setFont(fixedFont);

  setLineWrapMode(QPlainTextEdit::NoWrap);
}

void CodeEditor::enableSyntaxHighlighter(const std::string &name, std::string nameIs) {
#ifdef KF5_FOUND
  auto *highlighter = new KSyntaxHighlighting::SyntaxHighlighter(document());
  static auto repository = std::make_unique<KSyntaxHighlighting::Repository>();

  highlighter->setTheme(repository->defaultTheme(
    QApplication::palette().color(QPalette::Window).lightness() < 128 ? KSyntaxHighlighting::Repository::DarkTheme : KSyntaxHighlighting::Repository::LightTheme
  ));

  boost::algorithm::to_lower(nameIs);
  if(nameIs=="definitionname") {
    if(name=="EVALUATOR")
      highlighter->setDefinition(repository->definitionForName(Evaluator::getKDESyntaxHighlighterName().c_str()));
    else
      highlighter->setDefinition(repository->definitionForName(name.c_str()));
  }
  else if(nameIs=="filename")
    highlighter->setDefinition(repository->definitionForFileName(name.c_str()));
  else if(nameIs=="mimetype")
    highlighter->setDefinition(repository->definitionForMimeType(name.c_str()));
#endif
}

int CodeEditor::lineNumberAreaWidth() {
  int digits = 1;
  int max = qMax(1, blockCount());
  while(max >= 10) {
    max /= 10;
    ++digits;
  }

  int space = 3 + fontMetrics().horizontalAdvance(QLatin1Char('9')) * digits;

  return space;
}

void CodeEditor::updateLineNumberAreaWidth(int /* newBlockCount */) {
  setViewportMargins(lineNumberAreaWidth(), 0, 0, 0);
}

void CodeEditor::updateLineNumberArea(const QRect &rect, int dy) {
  if(dy)
    lineNumberArea->scroll(0, dy);
  else
    lineNumberArea->update(0, rect.y(), lineNumberArea->width(), rect.height());

  if(rect.contains(viewport()->rect()))
    updateLineNumberAreaWidth(0);
}

void CodeEditor::resizeEvent(QResizeEvent *e) {
  QPlainTextEdit::resizeEvent(e);

  QRect cr = contentsRect();
  lineNumberArea->setGeometry(QRect(cr.left(), cr.top(), lineNumberAreaWidth(), cr.height()));
}

void CodeEditor::lineNumberAreaPaintEvent(QPaintEvent *event) {
  QPainter painter(lineNumberArea);
  painter.fillRect(event->rect(), QApplication::palette().color(QPalette::Active, QPalette::Window)); // background color

  QTextBlock block = firstVisibleBlock();
  int blockNumber = block.blockNumber();
  int top = qRound(blockBoundingGeometry(block).translated(contentOffset()).top());
  int bottom = top + qRound(blockBoundingRect(block).height());

  while(block.isValid() && top <= event->rect().bottom()) {
    if(block.isVisible() && bottom >= event->rect().top()) {
      QString number = QString::number(blockNumber + 1);
      painter.setPen(QApplication::palette().color(QPalette::Disabled, QPalette::Text)); // foreground color
      painter.drawText(0, top, lineNumberArea->width(), fontMetrics().height(),
                       Qt::AlignRight, number);
    }

    block = block.next();
    top = bottom;
    bottom = top + qRound(blockBoundingRect(block).height());
    ++blockNumber;
  }
}

}
