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

#pragma once

#include <math.h>
#include <vector>
#include <utility>
#include <queue>
#include <algorithm>
#include "perception/localisation/Localiser.hpp"
#include "perception/localisation/LocalisationDefs.hpp"

/* Helps me findout where I am
*/
class PFilter : public Localiser {
   friend class OverviewTab;
   friend class LocalisationReader;
   friend class InverseVision;

   public:
   /**
    * Constructor for localisation module.
    **/
   explicit PFilter();
   /* Destructor */
   ~PFilter();

   // private:

   FieldLineLocalisation fieldLineLocalisation;
   std::vector<AbsCoord> robotLocation;

   /* first observation for the PF */
   bool firstObs;

   bool ourPost;

   Point topLeft;
   Point btmRight;

   unsigned int seed;

   /* a list of particles sorted by weight */
   std::vector<Particle> robotsPF;

   /** Particle Filter update */
   bool localise();

   /*** Private helper functions */
   /* Generate particles when PF first run */
   void generateParticles();

   /* Generate particles from posts if any */
   void generateFromPosts();

   /* Generate particles from field edges if any */
   void generateFromFieldEdges();

   /* Filter particles using the goal posts alone */
   void filterByGoalPosts();

   void filterByFieldEdges();

   /* Filter particles using field lines */
   void filterByFieldLines();

   void generateFromTwoEdges(Edge edges[], Line lines[],
         Point corner);

   void generateFromEdge(Edge edge, Line line);

   void getPosFromPosts();

   bool updateParticleByTwoPosts();

   bool updateParticleByPost(Particle *robot, int post);

   void normaliseWeights();

   void resetBoundingBox();

   void updateBoundingBox(Particle& particle);
};
