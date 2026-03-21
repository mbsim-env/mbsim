/* this code is taken from Qt examples */

#include <QPlainTextEdit>

class CodeEditor : public QPlainTextEdit {
  Q_OBJECT
  public:
    CodeEditor(QWidget *parent = nullptr);
    void lineNumberAreaPaintEvent(QPaintEvent *event);
    int lineNumberAreaWidth();
  
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
