/*
 * KdTree.cc
 * RVO2 Library
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/RVO2/>
 */

/**
 * @file  KdTree.cc
 * @brief Defines the KdTree class.
 */

#include "KdTree.h"

#include <algorithm>
#include <utility>

#include "Agent.h"
#include "Obstacle.h"
#include "RVOSimulator.h"
#include "Vector2.h"

namespace RVO {
namespace {
/**
 * @relates KdTree
 * @brief   The maximum k-D tree node leaf size.
 */
const std::size_t RVO_MAX_LEAF_SIZE = 10U;
} /* namespace */

/**
 * @brief Defines an agent k-D tree node.
 */
class KdTree::AgentTreeNode {
 public:
  /**
   * @brief Constructs an agent k-D tree node instance.
   */
  AgentTreeNode();

  /**
   * @brief The beginning node number.
   */
  std::size_t begin;

  /**
   * @brief The ending node number.
   */
  std::size_t end;

  /**
   * @brief The left node number.
   */
  std::size_t left;

  /**
   * @brief The right node number.
   */
  std::size_t right;

  /**
   * @brief The maximum x-coordinate.
   */
  float maxX;

  /**
   * @brief The maximum y-coordinate.
   */
  float maxY;

  /**
   * @brief The minimum x-coordinate.
   */
  float minX;

  /**
   * @brief The minimum y-coordinate.
   */
  float minY;
};

KdTree::AgentTreeNode::AgentTreeNode()
    : begin(0U),
      end(0U),
      left(0U),
      right(0U),
      maxX(0.0F),
      maxY(0.0F),
      minX(0.0F),
      minY(0.0F) {}

/**
 * @brief Defines an obstacle k-D tree node.
 */
class KdTree::ObstacleTreeNode {
 public:
  /**
   * @brief Constructs an obstacle k-D tree node instance.
   */
  ObstacleTreeNode();

  /**
   * @brief Destroys this obstacle k-D tree node instance.
   */
  ~ObstacleTreeNode();

  /**
   * @brief The obstacle number.
   */
  const Obstacle *obstacle;

  /**
   * @brief The left obstacle tree node.
   */
  ObstacleTreeNode *left;

  /**
   * @brief The right obstacle tree node.
   */
  ObstacleTreeNode *right;

 private:
  /* Not implemented. */
  ObstacleTreeNode(const ObstacleTreeNode &other);

  /* Not implemented. */
  ObstacleTreeNode &operator=(const ObstacleTreeNode &other);
};

KdTree::ObstacleTreeNode::ObstacleTreeNode()
    : obstacle(NULL), left(NULL), right(NULL) {}

KdTree::ObstacleTreeNode::~ObstacleTreeNode() {}

KdTree::KdTree(RVOSimulator *simulator)
    : obstacleTree_(NULL), simulator_(simulator) {}

KdTree::~KdTree() { deleteObstacleTree(obstacleTree_); }

void KdTree::buildAgentTree() {
  if (agents_.size() < simulator_->agents_.size()) {
    // 把simulator_->agents_后加进来的加入到agents_，simulator_->agents_比agents_长出来的后面几个元素就是新加进来的
    agents_.insert(agents_.end(),
                   simulator_->agents_.begin() +
                       static_cast<std::ptrdiff_t>(agents_.size()),
                   simulator_->agents_.end());
    // 扩容2*size-1，可以看出，leaf节点保存数据，root和internal为无数据节点
    agentTree_.resize(2U * agents_.size() - 1U);
  }

  if (!agents_.empty()) {
    buildAgentTreeRecursive(0U, agents_.size(), 0U);
  }
}

void KdTree::buildAgentTreeRecursive(std::size_t begin, std::size_t end,
                                     std::size_t node) {
  agentTree_[node].begin = begin;
  agentTree_[node].end = end;
  agentTree_[node].minX = agentTree_[node].maxX = agents_[begin]->position_.x();
  agentTree_[node].minY = agentTree_[node].maxY = agents_[begin]->position_.y();

  // 从begin到end，所有点的包围盒范围
  for (std::size_t i = begin + 1U; i < end; ++i) {
    agentTree_[node].maxX =
        std::max(agentTree_[node].maxX, agents_[i]->position_.x());
    agentTree_[node].minX =
        std::min(agentTree_[node].minX, agents_[i]->position_.x());
    agentTree_[node].maxY =
        std::max(agentTree_[node].maxY, agents_[i]->position_.y());
    agentTree_[node].minY =
        std::min(agentTree_[node].minY, agents_[i]->position_.y());
  }

  // 一个node空间内agent数量10个以内
  if (end - begin > RVO_MAX_LEAF_SIZE) {
    /* No leaf node. */
    // 把最长的边（x or y)一分为二
    const bool isVertical = agentTree_[node].maxX - agentTree_[node].minX >
                            agentTree_[node].maxY - agentTree_[node].minY;
    const float splitValue =
        0.5F * (isVertical ? agentTree_[node].maxX + agentTree_[node].minX
                           : agentTree_[node].maxY + agentTree_[node].minY);

    std::size_t left = begin;
    std::size_t right = end;

    // 计算splitValue两遍各有多少个agent
    while (left < right) {
      while (left < right &&
             (isVertical ? agents_[left]->position_.x()
                         : agents_[left]->position_.y()) < splitValue) {
        ++left;
      }

      while (right > left &&
             (isVertical ? agents_[right - 1U]->position_.x()
                         : agents_[right - 1U]->position_.y()) >= splitValue) {
        --right;
      }
      // 索引号在前面的比splitValue大，索引号在后面的比splitvalue小，对调一下
      if (left < right) {
        std::swap(agents_[left], agents_[right - 1U]);
        ++left;
        --right;
      }
    }

    if (left == begin) {
      ++left;
      ++right;
    }

    // 设置当前node左孩子和右孩子在agentTree_中的索引
    // agentTree_.length不是节点个数，而是2*agents_.size()-1，一个满二叉树的数量
    // 那么node的左孩子设置为node+1这个节点
    agentTree_[node].left = node + 1U;
    // 要给左孩子预留出来空间，这个空间占多少呢？
    // 左孩子有left-begin元素，一个满二叉树的节点数量为2*(left-begin)-1个
    // 也就是说把agentTree_[node].left作为root，则占用2*(left-begin)-1个
    // 所以agentTree_[node].right为(2*(left-begin)-1) + 1=2*(left-begin)
    // 最后在加上node
    agentTree_[node].right = node + 2U * (left - begin);

    buildAgentTreeRecursive(begin, left, agentTree_[node].left);
    buildAgentTreeRecursive(left, end, agentTree_[node].right);
  }
}

/// 构建Kdtree
void KdTree::buildObstacleTree() {
  deleteObstacleTree(obstacleTree_);

  const std::vector<Obstacle *> obstacles(simulator_->obstacles_);
  obstacleTree_ = buildObstacleTreeRecursive(obstacles);
}

KdTree::ObstacleTreeNode *KdTree::buildObstacleTreeRecursive(
    const std::vector<Obstacle *> &obstacles) {
  if (!obstacles.empty()) {
    ObstacleTreeNode *const node = new ObstacleTreeNode();

    std::size_t optimalSplit = 0U;
    std::size_t minLeft = obstacles.size();
    std::size_t minRight = obstacles.size();

    for (std::size_t i = 0U; i < obstacles.size(); ++i) {
      std::size_t leftSize = 0U;
      std::size_t rightSize = 0U;

      // obstacleI1->obstacleI2的向量
      const Obstacle *const obstacleI1 = obstacles[i];
      const Obstacle *const obstacleI2 = obstacleI1->next_;

      // 尝试obstacleI1->obstacleI2向量是否可作为最优分割线，向量左右顶点数越相近，越适合做分割线，也就是Kdtree的node
      /* Compute optimal split node. */
      for (std::size_t j = 0U; j < obstacles.size(); ++j) {
        // 这种判断情况下，obstacleJ1可能是obstacleI1->next_
        if (i != j) {
          const Obstacle *const obstacleJ1 = obstacles[j];
          const Obstacle *const obstacleJ2 = obstacleJ1->next_;

          // 求obstacleJ1是否在obstacleI1->obstacleI2的左边
          const float j1LeftOfI = leftOf(obstacleI1->point_, obstacleI2->point_,
                                         obstacleJ1->point_);
          // 求obstacleJ2是否在obstacleI1->obstacleI2的左边
          const float j2LeftOfI = leftOf(obstacleI1->point_, obstacleI2->point_,
                                         obstacleJ2->point_);

          // obstacleJ1和obstacleJ2都在obstacleI1->obstacleI2的左边
          if (j1LeftOfI >= -RVO_EPSILON && j2LeftOfI >= -RVO_EPSILON) {
            ++leftSize;
            // obstacleJ1和obstacleJ2都在obstacleI1->obstacleI2的右边
          } else if (j1LeftOfI <= RVO_EPSILON && j2LeftOfI <= RVO_EPSILON) {
            ++rightSize;
          // obstacleJ1、obstacleJ2在向量obstacleI1->obstacleI2的两侧
          } else {
            ++leftSize;
            ++rightSize;
          }

          // 左右的顶点个数已经不够平均了，break跳出
          if (std::make_pair(std::max(leftSize, rightSize),
                             std::min(leftSize, rightSize)) >=
              std::make_pair(std::max(minLeft, minRight),
                             std::min(minLeft, minRight))) {
            break;
          }
        }
      }

      // 如果发现左右顶点数量更平均的分割线，则记录i为最优分割线optimalSplit
      if (std::make_pair(std::max(leftSize, rightSize),
                         std::min(leftSize, rightSize)) <
          std::make_pair(std::max(minLeft, minRight),
                         std::min(minLeft, minRight))) {
        minLeft = leftSize;
        minRight = rightSize;
        optimalSplit = i;
      }
    }

    /* Build split node. */
    std::vector<Obstacle *> leftObstacles(minLeft);
    std::vector<Obstacle *> rightObstacles(minRight);

    std::size_t leftCounter = 0U;
    std::size_t rightCounter = 0U;
    const std::size_t i = optimalSplit;

    // 选中i为最优分割线
    const Obstacle *const obstacleI1 = obstacles[i];
    const Obstacle *const obstacleI2 = obstacleI1->next_;

    for (std::size_t j = 0U; j < obstacles.size(); ++j) {
      if (i != j) {
        Obstacle *const obstacleJ1 = obstacles[j];
        Obstacle *const obstacleJ2 = obstacleJ1->next_;

        const float j1LeftOfI =
            leftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ1->point_);
        const float j2LeftOfI =
            leftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ2->point_);

        if (j1LeftOfI >= -RVO_EPSILON && j2LeftOfI >= -RVO_EPSILON) {
          leftObstacles[leftCounter++] = obstacles[j];
        } else if (j1LeftOfI <= RVO_EPSILON && j2LeftOfI <= RVO_EPSILON) {
          rightObstacles[rightCounter++] = obstacles[j];
        } else {
          // 在两侧，则新产生一个点
          /* Split obstacle j. */
          const float t = det(obstacleI2->point_ - obstacleI1->point_,
                              obstacleJ1->point_ - obstacleI1->point_) /
                          det(obstacleI2->point_ - obstacleI1->point_,
                              obstacleJ1->point_ - obstacleJ2->point_);

          const Vector2 splitPoint =
              obstacleJ1->point_ +
              t * (obstacleJ2->point_ - obstacleJ1->point_);

          Obstacle *const newObstacle = new Obstacle();
          newObstacle->direction_ = obstacleJ1->direction_;
          newObstacle->point_ = splitPoint;
          newObstacle->next_ = obstacleJ2;
          newObstacle->previous_ = obstacleJ1;
          newObstacle->id_ = simulator_->obstacles_.size();
          newObstacle->isConvex_ = true;
          simulator_->obstacles_.push_back(newObstacle);

          obstacleJ1->next_ = newObstacle;
          obstacleJ2->previous_ = newObstacle;

          if (j1LeftOfI > 0.0F) {
            leftObstacles[leftCounter++] = obstacleJ1;
            rightObstacles[rightCounter++] = newObstacle;
          } else {
            rightObstacles[rightCounter++] = obstacleJ1;
            leftObstacles[leftCounter++] = newObstacle;
          }
        }
      }
    }

    // 设置obstacleI1为Kdtree的最优分割点
    node->obstacle = obstacleI1;
    // 左边、右边、继续递归
    node->left = buildObstacleTreeRecursive(leftObstacles);
    node->right = buildObstacleTreeRecursive(rightObstacles);

    return node;
  }

  return NULL;
}

void KdTree::computeAgentNeighbors(Agent *agent, float &rangeSq) const {
  queryAgentTreeRecursive(agent, rangeSq, 0U);
}

void KdTree::computeObstacleNeighbors(Agent *agent, float rangeSq) const {
  queryObstacleTreeRecursive(agent, rangeSq, obstacleTree_);
}

void KdTree::deleteObstacleTree(ObstacleTreeNode *node) {
  if (node != NULL) {
    deleteObstacleTree(node->left);
    deleteObstacleTree(node->right);
    delete node;
  }
}

void KdTree::queryAgentTreeRecursive(Agent *agent, float &rangeSq, std::size_t node) const {
  if (agentTree_[node].end - agentTree_[node].begin <= RVO_MAX_LEAF_SIZE) {
    for (std::size_t i = agentTree_[node].begin; i < agentTree_[node].end; ++i) {
      agent->insertAgentNeighbor(agents_[i], rangeSq);
    }
  } else {
    const float distLeftMinX = std::max(
        0.0F, agentTree_[agentTree_[node].left].minX - agent->position_.x());
    const float distLeftMaxX = std::max(
        0.0F, agent->position_.x() - agentTree_[agentTree_[node].left].maxX);
    const float distLeftMinY = std::max(
        0.0F, agentTree_[agentTree_[node].left].minY - agent->position_.y());
    const float distLeftMaxY = std::max(
        0.0F, agent->position_.y() - agentTree_[agentTree_[node].left].maxY);

    const float distSqLeft =
        distLeftMinX * distLeftMinX + distLeftMaxX * distLeftMaxX +
        distLeftMinY * distLeftMinY + distLeftMaxY * distLeftMaxY;

    const float distRightMinX = std::max(
        0.0F, agentTree_[agentTree_[node].right].minX - agent->position_.x());
    const float distRightMaxX = std::max(
        0.0F, agent->position_.x() - agentTree_[agentTree_[node].right].maxX);
    const float distRightMinY = std::max(
        0.0F, agentTree_[agentTree_[node].right].minY - agent->position_.y());
    const float distRightMaxY = std::max(
        0.0F, agent->position_.y() - agentTree_[agentTree_[node].right].maxY);

    const float distSqRight =
        distRightMinX * distRightMinX + distRightMaxX * distRightMaxX +
        distRightMinY * distRightMinY + distRightMaxY * distRightMaxY;

    if (distSqLeft < distSqRight) {
      if (distSqLeft < rangeSq) {
        queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);

        if (distSqRight < rangeSq) {
          queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);
        }
      }
    } else if (distSqRight < rangeSq) {
      queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);

      if (distSqLeft < rangeSq) {
        queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);
      }
    }
  }
}

void KdTree::queryObstacleTreeRecursive(Agent *agent, float rangeSq,
                                        const ObstacleTreeNode *node) const {
  if (node != NULL) {
    const Obstacle *const obstacle1 = node->obstacle;
    const Obstacle *const obstacle2 = obstacle1->next_;

    const float agentLeftOfLine =
        leftOf(obstacle1->point_, obstacle2->point_, agent->position_);

    queryObstacleTreeRecursive(
        agent, rangeSq, agentLeftOfLine >= 0.0F ? node->left : node->right);

    // 到线的距离的平方
    const float distSqLine = agentLeftOfLine * agentLeftOfLine /
                             absSq(obstacle2->point_ - obstacle1->point_);

    if (distSqLine < rangeSq) {
      // 在line的右边，为什么？？
      if (agentLeftOfLine < 0.0F) {
        // 仅当agent在障碍物右边且能看见障碍物时才加到Neighbor
        /* Try obstacle at this node only if agent is on right side of obstacle
         * and can see obstacle. */
        agent->insertObstacleNeighbor(node->obstacle, rangeSq);
      }

      /* Try other side of line. */
      queryObstacleTreeRecursive(
          agent, rangeSq, agentLeftOfLine >= 0.0F ? node->right : node->left);
    }
  }
}


bool KdTree::queryVisibility(const Vector2 &vector1, const Vector2 &vector2,
                             float radius) const {
  return queryVisibilityRecursive(vector1, vector2, radius, obstacleTree_);
}

/// 检查vector1是否能看见vector2
/// 何为看见？就是vector1和vector2之间没有障碍物，KdTree已经按照障碍物的边划分，在障碍物边的两侧即有可能互相看不见，在同侧则看得见
/// 按照KdTree递归二分空间查找，如果在所有二分查找的node分割线的同侧，直到已经没有子node了，则两点可见
bool KdTree::queryVisibilityRecursive(const Vector2 &vector1,
                                      const Vector2 &vector2, float radius,
                                      const ObstacleTreeNode *node) const {
  if (node != NULL) {
    const Obstacle *const obstacle1 = node->obstacle;
    const Obstacle *const obstacle2 = obstacle1->next_;

    const float q1LeftOfI =
        leftOf(obstacle1->point_, obstacle2->point_, vector1);
    const float q2LeftOfI =
        leftOf(obstacle1->point_, obstacle2->point_, vector2);
    // 距离的平方  分之一
    const float invLengthI =
        1.0F / absSq(obstacle2->point_ - obstacle1->point_);

    // q1LeftOfI * q1LeftOfI * invLengthI
    // 为vector1到obstacle1->obstacle2的距离
    // q1LeftOfI为平行四边形面积=底*高，invLengthI为底，则算出来的值为高，因为是用的是底的平方（invLengthI），所以都用平方

    // 都在分割线左边
    if (q1LeftOfI >= 0.0F && q2LeftOfI >= 0.0F) {
      // 两种情况满足条件
      // 1. vector1和vector2完全在该node分割线的左边，即q1LeftOfI * q1LeftOfI * invLengthI >= radius * radius && q2LeftOfI * q2LeftOfI * invLengthI >= radius * radius
      // 2. vector1和vector2因为有半径，没完全在node分割线左边，那么也要检查右侧，即queryVisibilityRecursive(vector1, vector2, radius, node->right)
      return queryVisibilityRecursive(vector1, vector2, radius, node->left) &&
             ((q1LeftOfI * q1LeftOfI * invLengthI >= radius * radius &&
               q2LeftOfI * q2LeftOfI * invLengthI >= radius * radius) ||
              queryVisibilityRecursive(vector1, vector2, radius, node->right));
    }

    if (q1LeftOfI <= 0.0F && q2LeftOfI <= 0.0F) {
      return queryVisibilityRecursive(vector1, vector2, radius, node->right) &&
             ((q1LeftOfI * q1LeftOfI * invLengthI >= radius * radius &&
               q2LeftOfI * q2LeftOfI * invLengthI >= radius * radius) ||
              queryVisibilityRecursive(vector1, vector2, radius, node->left));
    }

    // 这个是什么情况？从左边能看到右边？？怎么解释
    if (q1LeftOfI >= 0.0F && q2LeftOfI <= 0.0F) {
      /* One can see through obstacle from left to right. */
      return queryVisibilityRecursive(vector1, vector2, radius, node->left) &&
             queryVisibilityRecursive(vector1, vector2, radius, node->right);
    }

    // 不在障碍物同侧的情况

    const float point1LeftOfQ = leftOf(vector1, vector2, obstacle1->point_);
    const float point2LeftOfQ = leftOf(vector1, vector2, obstacle2->point_);
    const float invLengthQ = 1.0F / absSq(vector2 - vector1);

    // 检查vector1和vector2之间是否有障碍物
    return point1LeftOfQ * point2LeftOfQ >= 0.0F && // 障碍物在vector1和vector2的同侧 &&
           point1LeftOfQ * point1LeftOfQ * invLengthQ > radius * radius && // 障碍物obstacle1到vector1 vector1的距离超过radius &&
           point2LeftOfQ * point2LeftOfQ * invLengthQ > radius * radius && // 障碍物obstacle2到vector1 vector1的距离超过radius &&
           queryVisibilityRecursive(vector1, vector2, radius, node->left) && // 左树 &&
           queryVisibilityRecursive(vector1, vector2, radius, node->right); // 右树
  }

  // 当前叶子节点为null，说明已经没有障碍物阻碍vector1和vector2了，所以返回true，可见
  return true;
}
} /* namespace RVO */
