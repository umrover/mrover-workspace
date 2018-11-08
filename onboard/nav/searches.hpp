#ifndef SEARCHES_HPP
#define SEARCHES_HPP

#include "searcher.hpp"

enum class SearchType
{
	SPIRAL,
	LAWNMOWER,
	UNKNOWN
};

Searcher* SearchFactory( StateMachine* stateMachine, SearchType type );

/*************************************************************************/
/* Spiral Search */
/*************************************************************************/
class Spiral : public Searcher 
{
public:
	Spiral( StateMachine* stateMachine_ ) 
	: Searcher(stateMachine_) {}

	~Spiral();

	// Initializes the search ponit multipliers to be the intermost loop
 	// of the search.
	void initializeSearch( Rover* mPhoebe, const rapidjson::Document& mRoverConfig );
	
};

/*************************************************************************/
/* LawnMower Search */
/*************************************************************************/
class LawnMower : public Searcher 
{
public:
	LawnMower( StateMachine* stateMachine_ ) 
	: Searcher(stateMachine_) {}

	~LawnMower();

	// Initializes the search ponit multipliers to be the intermost loop
 	// of the search.
	void initializeSearch( Rover* mPhoebe, const rapidjson::Document& mRoverConfig );

};

#endif //SEARCHES_HPP