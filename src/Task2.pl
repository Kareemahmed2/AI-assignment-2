% Main solve predicate with energy limit
solve :-
    write('Enter the drone\'s initial energy (e.g., 6): '),
    read(X),
    integer(X),
    X > 0,
    initial_state(InitialState),
    goal_state(GoalState),
    calculateH(InitialState, GoalState, H),
    % Store initial energy for recharging
    retractall(solve_get_initial_energy(_)),
    assertz(solve_get_initial_energy(X)),
    % State representation now includes energy level: [State, Parent, G, H, F, Energy]
    search([ [InitialState, null, 0, H, H, X] ], [], GoalState).

% ========== STATE DEFINITIONS ==========
initial_state([
    [d, '-', p, '-', o],
    ['-', o, '-', '-', p],
    ['-', '-', o, p, '-'],
    [p, o, '-', 'R', '-'],  % Added recharge station R at position [3,3]
    ['-', '-', p, o, '-']
]).

% Goal state is determined by goal_condition/2
goal_state(GoalState) :- 
    initial_state(GoalState).

% ========== SEARCH CORE ==========
search(Open, Closed, Goal):-
    getBestState(Open, [CurrentState, Parent, G, H, F, Energy], TmpOpen),
    % Debug info can be uncommented to check energy levels
    % write('Current Energy: '), write(Energy), nl,
    goal_condition(CurrentState, Goal), !,
    write("Optimal Drone Route Found!"), nl, nl,
    solve_get_initial_energy(InitialEnergy),
    write("Initial Energy: "), write(InitialEnergy), nl, nl,
    printSolution([CurrentState, Parent, G, H, F, Energy], Closed),
    write("Final State:"), nl,
    printGrid(CurrentState).

search(Open, Closed, Goal):-
    getBestState(Open, CurrentNode, TmpOpen),
    getAllValidChildren(CurrentNode, TmpOpen, Closed, Goal, Children),
    addChildren(Children, TmpOpen, NewOpen),
    append(Closed, [CurrentNode], NewClosed),
    search(NewOpen, NewClosed, Goal).

% ========== GOAL CHECKING ==========
goal_condition(State, _) :-
    flatten(State, FlatState),
    \+ member(p, FlatState).

% ========== HEURISTIC FUNCTION ==========
calculateH(State, _, H):-
    flatten(State, FlatState),
    findall(X, (member(X, FlatState), X == p), Ps),
    length(Ps, H).

% ========== NODE SELECTION ==========
getBestState(Open, BestChild, Rest):-
    findMin(Open, BestChild),
    delete(Open, BestChild, Rest).

findMin([X], X):- !.
findMin([Head|T], Min):-
    findMin(T, TmpMin),
    Head = [_, _, _, _, HeadF, _],
    TmpMin = [_, _, _, _, TmpF, _],
    (TmpF < HeadF -> Min = TmpMin ; Min = Head).

% ========== CHILD NODE GENERATION ==========
getAllValidChildren(Node, Open, Closed, Goal, Children):-
    findall(Next, getNextState(Node, Open, Closed, Goal, Next), Children).

% Updated to account for energy constraints
getNextState([State, Parent, G, _, _, Energy], Open, Closed, Goal, [Next, State, NewG, NewH, NewF, NewEnergy]):-
    Energy > 0,  % Only allow moves if there's energy left
    move(State, Next, MoveCost),
    % Calculate new energy before the move (will be updated if at recharge)
    TempEnergy is Energy - MoveCost,
    % Check if drone is at recharge station after the move
    (is_drone_at_recharge(Next) -> 
        % Get max energy (same as initial energy)
        solve_get_initial_energy(MaxEnergy),
        NewEnergy = MaxEnergy
    ;
        % Normal energy reduction
        NewEnergy = TempEnergy
    ),
    
    % Energy must be non-negative after the move
    NewEnergy >= 0,

    % Standard A* calculation
    calculateH(Next, Goal, NewH),
    NewG is G + MoveCost,
    NewF is NewG + NewH,
    
    % Standard A* checks
    (not(member([Next, _, _, _, _, _], Open)) ; memberButBetter(Next, Open, NewF, NewEnergy)),
    (not(member([Next, _, _, _, _, _], Closed)) ; memberButBetter(Next, Closed, NewF, NewEnergy)).

% Check if drone is currently at a recharge station
is_drone_at_recharge(State) :-
    find_drone(State, (R, C)),
    initial_state(InitState),
    nth0(R, InitState, InitRow),
    nth0(C, InitRow, Cell),
    Cell == 'R'.

% Helper to get initial energy (for recharge)
:- dynamic solve_get_initial_energy/1.
solve_get_initial_energy(6).  % Default value, will be updated during solve

% Updated to consider energy in comparison
memberButBetter(Next, List, NewF, NewEnergy):-
    findall([F, E], member([Next, _, _, _, F, E], List), Values),
    (Values = [] -> true ; 
        better_state(NewF, NewEnergy, Values)).

% A state is better if it has either lower F or same F with more energy
better_state(NewF, NewEnergy, Values) :-
    \+ (member([F, E], Values), (F < NewF ; (F =:= NewF, E >= NewEnergy))).

addChildren(Children, Open, NewOpen):-
    append(Open, Children, NewOpen).

% ========== SOLUTION PRINTING ==========
printSolution([State, null, G, H, F, Energy], _):-
    write("Initial State (g="), write(G), write(", h="), write(H), 
    write(", f="), write(F), write(", energy="), write(Energy), write("):"), nl,
    printGrid(State), nl, nl,
    write("Steps:"), nl, nl.

printSolution([State, Parent, G, H, F, Energy], Closed):-
    member([Parent, GrandParent, PrevG, PH, PF, PEnergy], Closed),
    printSolution([Parent, GrandParent, PrevG, PH, PF, PEnergy], Closed),
    write("Move (g="), write(G), write(", h="), write(H), 
    write(", f="), write(F), write(", energy="), write(Energy), write("):"), nl,
    printGrid(State), nl.

printGrid([]).
printGrid([H|T]) :-
    write('    '),
    write(H), nl,
    printGrid(T).

% ========== MOVEMENT RULES ==========
move(State, Next, 1):-
    left(State, Next); right(State, Next); 
    up(State, Next); down(State, Next).

find_drone(State, (Row, Col)) :-
    nth0(Row, State, RowList),
    nth0(Col, RowList, d).

% Left movement
left(State, Next):-
    find_drone(State, (R, C)),
    C > 0,
    NewC is C - 1,
    nth0(R, State, Row),
    nth0(NewC, Row, Cell),
    Cell \= o,
    mark_position(State, R, C, NewC, Next).

% Right movement
right(State, Next):-
    find_drone(State, (R, C)),
    nth0(R, State, Row),
    length(Row, Cols),
    C < Cols - 1,
    NewC is C + 1,
    nth0(NewC, Row, Cell),
    Cell \= o,
    mark_position(State, R, C, NewC, Next).

% Up movement
up(State, Next):-
    find_drone(State, (R, C)),
    R > 0,
    NewR is R - 1,
    nth0(NewR, State, NewRow),
    nth0(C, NewRow, Cell),
    Cell \= o,
    mark_position_row(State, R, C, NewR, Next).

% Down movement
down(State, Next):-
    find_drone(State, (R, C)),
    length(State, Rows),
    R < Rows - 1,
    NewR is R + 1,
    nth0(NewR, State, NewRow),
    nth0(C, NewRow, Cell),
    Cell \= o,
    mark_position_row(State, R, C, NewR, Next).

% ========== GRID MODIFICATION ==========
% Modified to handle recharge stations - horizontal movement
mark_position(State, R, C, NewC, Next) :-
    % Check if current position is a recharge station in the initial state
    initial_state(InitState),
    nth0(R, InitState, InitRow),
    nth0(C, InitRow, CellType),
    
    % If current cell was a recharge station in initial state, restore R, otherwise empty
    (CellType == 'R' -> 
        substitute_in_grid(State, R, C, 'R', TempState)
    ;
        substitute_in_grid(State, R, C, '-', TempState)
    ),
    
    % Handle destination cell, preserving special cells
    nth0(R, TempState, TempRow),
    nth0(NewC, TempRow, DestContent),
    
    % Place drone in the destination, preserving package pickup logic
    substitute_in_grid(TempState, R, NewC, d, Next).

% Modified to handle recharge stations - vertical movement
mark_position_row(State, R, C, NewR, Next) :-
    % Check if current position is a recharge station in the initial state
    initial_state(InitState),
    nth0(R, InitState, InitRow),
    nth0(C, InitRow, CellType),
    
    % If current cell was a recharge station in initial state, restore R, otherwise empty
    (CellType == 'R' -> 
        substitute_in_grid(State, R, C, 'R', TempState)
    ;
        substitute_in_grid(State, R, C, '-', TempState)
    ),
    
    % Handle destination cell, preserving special cells
    nth0(NewR, TempState, TempRow),
    nth0(C, TempRow, DestContent),
    
    % Place drone in the destination, preserving package pickup logic
    substitute_in_grid(TempState, NewR, C, d, Next).

% Grid manipulation predicates
substitute_in_grid(Grid, Row, Col, NewElement, NewGrid) :-
    nth0(Row, Grid, OldRow),
    substitute_in_row(OldRow, Col, NewElement, NewRow),
    replace_nth0(Row, Grid, NewRow, NewGrid).

substitute_in_row(Row, Col, NewElement, NewRow) :-
    replace_nth0(Col, Row, NewElement, NewRow).

% Helper predicate to replace element at index N in a list
replace_nth0(0, [_|T], X, [X|T]) :- !.
replace_nth0(N, [H|T], X, [H|R]) :-
    N > 0,
    N1 is N - 1,
    replace_nth0(N1, T, X, R).