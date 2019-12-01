/**
 * Copyright (c) 2019, Felix Morgner <felix.morgner@gmail.com>, all rights reserved
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 *  - Neither the name of the copyright holders nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// A solution for Jason Turners 'Constexpr Maze'

#include "maze_generator.hpp"

#include <array>
#include <cstddef>
#include <tuple>

/**
 * Check if the given command can be performed given the current player location
 * 
 * @note Command::Stay is always a valid option
 */
template <typename MapType>
auto constexpr can_move(MapType map, Command command, Loc position)
{
  switch (command)
  {
  case Command::Left:
    return (position.col - 1 > 0) && (map(position.col - 1, position.row) == WallType::Empty);
  case Command::Right:
    return (position.col + 1 < map.cols()) && (map(position.col + 1, position.row) == WallType::Empty);
  case Command::Up:
    return (position.row - 1 > 0) && (map(position.col, position.row - 1) == WallType::Empty);
  case Command::Down:
    return (position.row + 1 < map.rows()) && (map(position.col, position.row + 1) == WallType::Empty);
  }
  return true;
}

/**
 * Calculate the player location resulting from applying the given command
 */
auto constexpr update_position(Command move, Loc position)
{
  auto [column, row] = position;
  switch (move)
  {
  case Command::Right:
    return Loc{column + 1, row};
  case Command::Left:
    return Loc{column - 1, row};
  case Command::Up:
    return Loc{column, row - 1};
  case Command::Down:
    return Loc{column, row + 1};
  }
  return position;
}

/**
 * Calculate the inverse of a given move
 */
auto constexpr invert_move(Command move)
{
  switch (move)
  {
  case Command::Right:
    return Command::Left;
  case Command::Left:
    return Command::Right;
  case Command::Up:
    return Command::Down;
  case Command::Down:
    return Command::Up;
  }
  return Command::Stay;
}

/**
 * Calculate the new route and player position resulting from applying the given command
 */
template <typename RouteType>
auto constexpr do_move(RouteType route, Command move, Loc position)
{
  route.emplace_back(move);
  return std::tuple(route, update_position(move, position));
}

/**
 * Calculate the new route and player position resulting from reversing the last command in the route
 */
template <typename RouteType>
auto constexpr undo_move(RouteType route, Loc position)
{
  auto prior_move = route.pop_back();
  auto inverse_move = invert_move(prior_move);
  return std::tuple(route, update_position(inverse_move, position));
}

/**
 * Find a path through the maze
 */
template <
    typename FieldType,
    std::size_t Columns,
    std::size_t Rows>
auto constexpr find_route(Array2D<FieldType, Columns, Rows> map, Loc player, Loc goal)
{
  auto route = Stack<Command, Columns * Rows>{};

  while (player.col != goal.col || player.row != goal.row)
  {
    map(player.col, player.row) = WallType::Visited;

    auto [new_route, new_position] = [=] {
      if (can_move(map, Command::Right, player))
      {
        return do_move(route, Command::Right, player);
      }
      else if (can_move(map, Command::Down, player))
      {
        return do_move(route, Command::Down, player);
      }
      else if (can_move(map, Command::Left, player))
      {
        return do_move(route, Command::Left, player);
      }
      else if (can_move(map, Command::Up, player))
      {
        return do_move(route, Command::Up, player);
      }
      else if (route.size() > 0)
      {
        return undo_move(route, player);
      }
      else
      {
        return std::tuple{route, player};
      }
    }();

    route = new_route;
    player = new_position;
  }

  return route;
}

/**
 * Convert a route stack into an array of commands
 */
template <size_t Moves, typename RouteType>
auto constexpr to_command_array(RouteType route)
{
  auto moves = std::array<Command, Moves>{};
  auto route_length = route.size();

  for (auto move = std::size_t{}; move < Moves; ++move)
  {
    moves[Moves - move - 1] = route.pop_back();
  }

  return moves;
}

int main()
{
  auto constexpr num_cols = std::size_t{15};
  auto constexpr num_rows = std::size_t{8};
  auto constexpr seed = std::size_t{1024};

  auto constexpr maze = add_monsters(render_maze(make_maze<num_cols, num_rows>(seed)), seed);
  auto constexpr route = find_route(maze, {0, 1}, {maze.cols() - 1, maze.rows() - 2});
  auto constexpr commands = to_command_array<route.size()>(route);

  print_maze(execute_commands(maze, commands));
}
