/* this code is taken from Qt examples */

#ifndef MBSIMGUI_CODEEDITOR_H
#define MBSIMGUI_CODEEDITOR_H

#include <QPlainTextEdit>

namespace MBSimGUI {

class CodeEditor : public QPlainTextEdit {
  Q_OBJECT
  public:
    CodeEditor(QWidget *parent = nullptr);
    void lineNumberAreaPaintEvent(QPaintEvent *event);
    int lineNumberAreaWidth();

    /** Enables syntax highlighting.
     * name: the definition name of the highlighter, if nameIs=="definitionName"
     *       the mimetype name of the highlighter, if nameIs=="mimeType"
     *       the highlighter which corresponds to the filename name, if nameIs=="filename"
     */
    void enableSyntaxHighlighter(const std::string &name="EVALUATOR", std::string nameIs="definitionName");
  
  protected:
    void resizeEvent(QResizeEvent *event) override;
  
  private Q_SLOTS:
    void updateLineNumberAreaWidth(int newBlockCount);
    void updateLineNumberArea(const QRect &rect, int dy);
  
  private:
    QWidget *lineNumberArea;
};

class LineNumberArea : public QWidget {
  public:
    LineNumberArea(CodeEditor *editor) : QWidget(editor), codeEditor(editor)
    {}
  
    QSize sizeHint() const override {
      return QSize(codeEditor->lineNumberAreaWidth(), 0);
    }
  
  protected:
    void paintEvent(QPaintEvent *event) override {
      codeEditor->lineNumberAreaPaintEvent(event);
    }
  
  private:
    CodeEditor *codeEditor;
};

}

#endif
