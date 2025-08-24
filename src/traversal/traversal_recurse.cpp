/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jia Pan */

#include "coal/internal/traversal_recurse.h"

#include <vector>

namespace coal {
void collisionRecurse(CollisionTraversalNodeBase* node, unsigned int b1,
                      unsigned int b2, BVHFrontList* front_list,
                      Scalar& sqrDistLowerBound) {
  Scalar sqrDistLowerBound1 = 0, sqrDistLowerBound2 = 0;
  bool l1 = node->isFirstNodeLeaf(b1);
  bool l2 = node->isSecondNodeLeaf(b2);
  if (l1 && l2) {
    updateFrontList(front_list, b1, b2);

    // if(node->BVDisjoints(b1, b2, sqrDistLowerBound)) return;
    node->leafCollides(b1, b2, sqrDistLowerBound);
    return;
  }

  if (node->BVDisjoints(b1, b2, sqrDistLowerBound)) {
    updateFrontList(front_list, b1, b2);
    return;
  }
  if (node->firstOverSecond(b1, b2)) {
    unsigned int c1 = (unsigned int)node->getFirstLeftChild(b1);
    unsigned int c2 = (unsigned int)node->getFirstRightChild(b1);

    collisionRecurse(node, c1, b2, front_list, sqrDistLowerBound1);

    // early stop is disabled is front_list is used
    if (node->canStop() && !front_list) return;

    collisionRecurse(node, c2, b2, front_list, sqrDistLowerBound2);
    sqrDistLowerBound = std::min(sqrDistLowerBound1, sqrDistLowerBound2);
  } else {
    unsigned int c1 = (unsigned int)node->getSecondLeftChild(b2);
    unsigned int c2 = (unsigned int)node->getSecondRightChild(b2);

    collisionRecurse(node, b1, c1, front_list, sqrDistLowerBound1);

    // early stop is disabled is front_list is used
    if (node->canStop() && !front_list) return;

    collisionRecurse(node, b1, c2, front_list, sqrDistLowerBound2);
    sqrDistLowerBound = std::min(sqrDistLowerBound1, sqrDistLowerBound2);
  }
}

void collisionNonRecurse(CollisionTraversalNodeBase* node,
                         BVHFrontList* front_list, Scalar& sqrDistLowerBound) {
  typedef std::pair<unsigned int, unsigned int> BVPair_t;
  // typedef std::stack<BVPair_t, std::vector<BVPair_t> > Stack_t;
  typedef std::vector<BVPair_t> Stack_t;

  Stack_t pairs;
  pairs.reserve(1000);
  sqrDistLowerBound = std::numeric_limits<Scalar>::infinity();
  Scalar sdlb = std::numeric_limits<Scalar>::infinity();

  pairs.push_back(BVPair_t(0, 0));

  while (!pairs.empty()) {
    unsigned int a = pairs.back().first, b = pairs.back().second;
    pairs.pop_back();

    bool la = node->isFirstNodeLeaf(a);
    bool lb = node->isSecondNodeLeaf(b);

    // Leaf / Leaf case
    if (la && lb) {
      updateFrontList(front_list, a, b);

      // TODO should we test the BVs ?
      // if(node->BVDijsoints(a, b, sdlb)) {
      // if (sdlb < sqrDistLowerBound) sqrDistLowerBound = sdlb;
      // continue;
      //}
      node->leafCollides(a, b, sdlb);
      if (sdlb < sqrDistLowerBound) sqrDistLowerBound = sdlb;
      if (node->canStop() && !front_list) return;
      continue;
    }

    // TODO shouldn't we test the leaf triangle against BV is la != lb
    // if (la && !lb) { // leaf triangle 1 against BV 2
    // } else if (!la && lb) { // BV 1 against leaf triangle 2
    // }

    // Check the BV
    if (node->BVDisjoints(a, b, sdlb)) {
      if (sdlb < sqrDistLowerBound) sqrDistLowerBound = sdlb;
      updateFrontList(front_list, a, b);
      continue;
    }

    if (node->firstOverSecond(a, b)) {
      unsigned int c1 = (unsigned int)node->getFirstLeftChild(a);
      unsigned int c2 = (unsigned int)node->getFirstRightChild(a);
      pairs.push_back(BVPair_t(c2, b));
      pairs.push_back(BVPair_t(c1, b));
    } else {
      unsigned int c1 = (unsigned int)node->getSecondLeftChild(b);
      unsigned int c2 = (unsigned int)node->getSecondRightChild(b);
      pairs.push_back(BVPair_t(a, c2));
      pairs.push_back(BVPair_t(a, c1));
    }
  }
}

/** Recurse function for self collision
 * Make sure node is set correctly so that the first and second tree are the
 * same
 */
void distanceRecurse(DistanceTraversalNodeBase* node, unsigned int b1,
                     unsigned int b2, BVHFrontList* front_list) {
  bool l1 = node->isFirstNodeLeaf(b1);
  bool l2 = node->isSecondNodeLeaf(b2);

  if (l1 && l2) {
    updateFrontList(front_list, b1, b2);

    node->leafComputeDistance(b1, b2);
    return;
  }

  unsigned int a1, a2, c1, c2;

  if (node->firstOverSecond(b1, b2)) {
    a1 = (unsigned int)node->getFirstLeftChild(b1);
    a2 = b2;
    c1 = (unsigned int)node->getFirstRightChild(b1);
    c2 = b2;
  } else {
    a1 = b1;
    a2 = (unsigned int)node->getSecondLeftChild(b2);
    c1 = b1;
    c2 = (unsigned int)node->getSecondRightChild(b2);
  }

  Scalar d1 = node->BVDistanceLowerBound(a1, a2);
  Scalar d2 = node->BVDistanceLowerBound(c1, c2);

  if (d2 < d1) {
    if (!node->canStop(d2))
      distanceRecurse(node, c1, c2, front_list);
    else
      updateFrontList(front_list, c1, c2);

    if (!node->canStop(d1))
      distanceRecurse(node, a1, a2, front_list);
    else
      updateFrontList(front_list, a1, a2);
  } else {
    if (!node->canStop(d1))
      distanceRecurse(node, a1, a2, front_list);
    else
      updateFrontList(front_list, a1, a2);

    if (!node->canStop(d2))
      distanceRecurse(node, c1, c2, front_list);
    else
      updateFrontList(front_list, c1, c2);
  }
}

/** @brief Bounding volume test structure */
struct COAL_LOCAL BVT {
  /** @brief distance between bvs */
  Scalar d;

  /** @brief bv indices for a pair of bvs in two models */
  unsigned int b1, b2;
};

/** @brief Comparer between two BVT */
struct COAL_LOCAL BVT_Comparer {
  bool operator()(const BVT& lhs, const BVT& rhs) const {
    return lhs.d > rhs.d;
  }  // namespace coal
};

struct COAL_LOCAL BVTQ {
  BVTQ() : qsize(2) {}

  bool empty() const { return pq.empty(); }

  size_t size() const { return pq.size(); }

  const BVT& top() const { return pq.top(); }

  void push(const BVT& x) { pq.push(x); }

  void pop() { pq.pop(); }

  bool full() const { return (pq.size() + 1 >= qsize); }

  std::priority_queue<BVT, std::vector<BVT>, BVT_Comparer> pq;

  /** @brief Queue size */
  unsigned int qsize;
};

void distanceQueueRecurse(DistanceTraversalNodeBase* node, unsigned int b1,
                          unsigned int b2, BVHFrontList* front_list,
                          unsigned int qsize) {
  BVTQ bvtq;
  bvtq.qsize = qsize;

  BVT min_test;
  min_test.b1 = b1;
  min_test.b2 = b2;

  while (1) {
    bool l1 = node->isFirstNodeLeaf(min_test.b1);
    bool l2 = node->isSecondNodeLeaf(min_test.b2);

    if (l1 && l2) {
      updateFrontList(front_list, min_test.b1, min_test.b2);

      node->leafComputeDistance(min_test.b1, min_test.b2);
    } else if (bvtq.full()) {
      // queue should not get two more tests, recur

      distanceQueueRecurse(node, min_test.b1, min_test.b2, front_list, qsize);
    } else {
      // queue capacity is not full yet
      BVT bvt1, bvt2;

      if (node->firstOverSecond(min_test.b1, min_test.b2)) {
        unsigned int c1 = (unsigned int)node->getFirstLeftChild(min_test.b1);
        unsigned int c2 = (unsigned int)node->getFirstRightChild(min_test.b1);
        bvt1.b1 = c1;
        bvt1.b2 = min_test.b2;
        bvt1.d = node->BVDistanceLowerBound(bvt1.b1, bvt1.b2);

        bvt2.b1 = c2;
        bvt2.b2 = min_test.b2;
        bvt2.d = node->BVDistanceLowerBound(bvt2.b1, bvt2.b2);
      } else {
        unsigned int c1 = (unsigned int)node->getSecondLeftChild(min_test.b2);
        unsigned int c2 = (unsigned int)node->getSecondRightChild(min_test.b2);
        bvt1.b1 = min_test.b1;
        bvt1.b2 = c1;
        bvt1.d = node->BVDistanceLowerBound(bvt1.b1, bvt1.b2);

        bvt2.b1 = min_test.b1;
        bvt2.b2 = c2;
        bvt2.d = node->BVDistanceLowerBound(bvt2.b1, bvt2.b2);
      }

      bvtq.push(bvt1);
      bvtq.push(bvt2);
    }

    if (bvtq.empty())
      break;
    else {
      min_test = bvtq.top();
      bvtq.pop();

      if (node->canStop(min_test.d)) {
        updateFrontList(front_list, min_test.b1, min_test.b2);
        break;
      }
    }
  }
}

void propagateBVHFrontListCollisionRecurse(CollisionTraversalNodeBase* node,
                                           const CollisionRequest& /*request*/,
                                           CollisionResult& result,
                                           BVHFrontList* front_list) {
  Scalar sqrDistLowerBound = -1, sqrDistLowerBound1 = 0, sqrDistLowerBound2 = 0;
  BVHFrontList::iterator front_iter;
  BVHFrontList append;
  for (front_iter = front_list->begin(); front_iter != front_list->end();
       ++front_iter) {
    unsigned int b1 = front_iter->left;
    unsigned int b2 = front_iter->right;
    bool l1 = node->isFirstNodeLeaf(b1);
    bool l2 = node->isSecondNodeLeaf(b2);

    if (l1 & l2) {
      front_iter->valid = false;  // the front node is no longer valid, in
                                  // collideRecurse will add again.
      collisionRecurse(node, b1, b2, &append, sqrDistLowerBound);
    } else {
      if (!node->BVDisjoints(b1, b2, sqrDistLowerBound)) {
        front_iter->valid = false;
        if (node->firstOverSecond(b1, b2)) {
          unsigned int c1 = (unsigned int)node->getFirstLeftChild(b1);
          unsigned int c2 = (unsigned int)node->getFirstRightChild(b1);

          collisionRecurse(node, c1, b2, front_list, sqrDistLowerBound1);
          collisionRecurse(node, c2, b2, front_list, sqrDistLowerBound2);
          sqrDistLowerBound = std::min(sqrDistLowerBound1, sqrDistLowerBound2);
        } else {
          unsigned int c1 = (unsigned int)node->getSecondLeftChild(b2);
          unsigned int c2 = (unsigned int)node->getSecondRightChild(b2);

          collisionRecurse(node, b1, c1, front_list, sqrDistLowerBound1);
          collisionRecurse(node, b1, c2, front_list, sqrDistLowerBound2);
          sqrDistLowerBound = std::min(sqrDistLowerBound1, sqrDistLowerBound2);
        }
      }
    }
    result.updateDistanceLowerBound(sqrt(sqrDistLowerBound));
  }

  // clean the old front list (remove invalid node)
  for (front_iter = front_list->begin(); front_iter != front_list->end();) {
    if (!front_iter->valid)
      front_iter = front_list->erase(front_iter);
    else
      ++front_iter;
  }

  for (front_iter = append.begin(); front_iter != append.end(); ++front_iter) {
    front_list->push_back(*front_iter);
  }
}

}  // namespace coal
