/*
Copyright 2010 The University of New South Wales (UNSW).

This file is part of the 2010 team rUNSWift RoboCup entry. You may
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version as
modified below. As the original licensors, we add the following
conditions to that license:

In paragraph 2.b), the phrase "distribute or publish" should be
interpreted to include entry into a competition, and hence the source
of any derived work entered into a competition must be made available
to all parties involved in that competition under the terms of this
license.

In addition, if the authors of a derived work publish any conference
proceedings, journal articles or other academic papers describing that
derived work, then appropriate academic citations to the original work
must be included in that publication.

This rUNSWift source is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with this source code; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <QtGui/QApplication>
#include <iostream>
#include "visualiser.hpp"
#include "utils/options.hpp"

namespace po = boost::program_options;
using namespace std;
using namespace boost;

po::variables_map config;

int main(int argc, char *argv[]) {
   /** Load options from config file into global 'config' variable */
   po::options_description config_file_options;
   populate_options(config_file_options);
   ifstream ifs;
   ifs.open("../runswift.cfg");
   store(parse_config_file(ifs, config_file_options), config);
   ifs.close();
   ifs.open("/etc/runswift/runswift.cfg");
   store(parse_config_file(ifs, config_file_options), config);
   ifs.close();
   po::notify(config);

   cerr << "logpath is: " << config["debug.logpath"].as<string>() << endl;

   /** Start the QT application */
   QApplication a(argc, argv);
   Visualiser w;
   w.show();
   return a.exec();
}
