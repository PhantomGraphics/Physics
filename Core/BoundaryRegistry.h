#pragma once

#include <algorithm>
#include <vector>

namespace Phantom {
namespace Physics {

/**
 * @brief Helpers for the non-owning boundary registration lists that
 * DFSPH/PBSPH/WCSPH keep (rigid boundaries, rigid/soft boundary particles).
 *
 * Contract shared by every ISPHSolver::add*()/remove*() pair:
 *  - nullptr is rejected (returns false), never stored;
 *  - registering a pointer that is already in the list is a no-op (false),
 *    so the same pointer can never be stepped twice;
 *  - removal drops exactly the given pointer and reports whether it was there.
 */
template <class Stored, class Given>
bool registerBoundary(std::vector<Stored*>& list, Given* p)
{
	if (p == nullptr) return false;
	Stored* s = p;
	if (std::find(list.begin(), list.end(), s) != list.end()) return false;
	list.push_back(s);
	return true;
}

template <class Stored, class Given>
bool unregisterBoundary(std::vector<Stored*>& list, Given* p)
{
	if (p == nullptr) return false;
	Stored* s = p;
	const auto it = std::find(list.begin(), list.end(), s);
	if (it == list.end()) return false;
	list.erase(it);
	return true;
}

} // namespace Physics
} // namespace Phantom
