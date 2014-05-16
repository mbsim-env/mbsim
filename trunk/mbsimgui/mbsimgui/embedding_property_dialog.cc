/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <config.h>
#include "embedding_property_dialog.h"
#include "element.h"
#include "basic_widgets.h"

using namespace std;

namespace MBSimGUI {

  EmbeddingPropertyDialog::EmbeddingPropertyDialog(Element *element_, bool embedding, QWidget *parent, Qt::WindowFlags f) : PropertyDialog(parent,f), element(element_), embed(0) {
    addTab("Embedding");
    if(embedding) {
      embed = new ExtWidget("Embed", new EmbedWidget, true);
      addToTab("Embedding",embed);
      name = new ExtWidget("Name",new TextWidget);
      addToTab("Embedding",name);
    }
  }

  void EmbeddingPropertyDialog::toWidget(Element *element) {
    if(embed) {
      element->embed.toWidget(embed);
      element->name.toWidget(name);
    }
  }

  void EmbeddingPropertyDialog::fromWidget(Element *element) {
    if(embed) {
      element->embed.fromWidget(embed);
      element->name.fromWidget(name);
    }
  }


}
